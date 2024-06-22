;______________________________________________________________________________;    
;    
;   File name: RTC.s
;   Version: 1.0   
;   Author: OzTheMaker
;
;______________________________________________________________________________;
;
;   Microcontroller compatibility: PIC16F506
;   Assembler: PIC-asm 
;   Description: 
;		  This is the code for the real time clock proyect presented in
;                 the OzTheMaker social media. 
;		  		 
;  
;    Pin assignment:
;
;	            ____________
;	           |            |
;	   VDD >---|	        |---< VSS
;	   DP1 <---|RB5      RB0|---> DP3
;	   DP2 <---|RB4      RB1|---> DP4
;	   SW1 >---|RB3      RB2|---< SW2
;          SCL >--<|RC5      RC0|---> A
;          SDA >--<|RC4      RC1|---> B
;            D <---|RC3      RC2|---> C
;                  |____________|
;
;______________________________________________________________________________;

PROCESSOR 16F506
#include <xc.inc>
        
;Variable definitions
DISPLAY_A EQU 0x0D
DISPLAY_B EQU 0x0E
DISPLAY_C EQU 0x0F
DISPLAY_D EQU 0x10
COUNTER1 EQU 0X11
COUNTER2 EQU 0X12 
MIN	EQU 0X13
HOUR	EQU 0X14
DAY	EQU 0X15
MONTH	EQU 0X16
YEAR	EQU 0X17
SDA EQU 0X04
SCL EQU 0X05
SW1 EQU 0X03
SW2 EQU 0X02 
BIT_COUNT EQU 0X18
I2C_DATA EQU 0X19 
PORT EQU 0X1A
ACK EQU 0X1B
 
 
 ;CONFIG word
CONFIG WDT = OFF
CONFIG CP = OFF
CONFIG MCLRE = OFF
CONFIG OSC = 100B ;Internal clock configured with port RB4 function.
CONFIG IOSCFS = 0
 
;Program memory reset vector
PSECT resetOrigin, class=CODE, delta=2

;Pin configuration
INIT:
    MOVLW ~(1 << CM1CON0_C1ON_POSITION)
    MOVWF CM1CON0
    MOVLW ~((1 << CM2CON0_C2PREF1_POSITION) | (1 << CM2CON0_C2ON_POSITION))
    MOVWF CM2CON0
    MOVLW ~((1 << ADCON0_ANS0_POSITION) | (1 << ADCON0_ANS1_POSITION))
    MOVWF ADCON0
    MOVLW ~((1 << PORTB_RB0_POSITION) | (1 << PORTB_RB1_POSITION) | (1 << PORTB_RB4_POSITION) | (1 << PORTB_RB5_POSITION))
    TRIS PORTB
    CLRF PORTB
    MOVLW ~(1 << VRCON_VROE_POSITION)
    MOVWF VRCON
    MOVLW ~((1 << PORTC_RC0_POSITION) | (1 << PORTC_RC1_POSITION) | (1 << PORTC_RC2_POSITION) | (1 << PORTC_RC3_POSITION))
    TRIS PORTC

;Code    
START:
    MOVLW 0X08
    MOVWF DISPLAY_A
    MOVWF DISPLAY_B
    MOVWF DISPLAY_C
    MOVWF DISPLAY_D

LOOP:
    CALL DISPLAY
    GOTO MAIN
    ;create routine to poll SW2
    






DISPLAY:
    MOVF DISPLAY_A, W
    MOVWF PORTC
    BSF PORTB, PORTB_RB5_POSITION
    CALL DELAY
    BCF PORTB, PORTB_RB5_POSITION

    MOVF DISPLAY_B, W
    MOVWF PORTC
    BSF PORTB, PORTB_RB4_POSITION
    CALL DELAY
    BCF PORTB, PORTB_RB4_POSITION

    MOVF DISPLAY_C, W
    MOVWF PORTC
    BSF PORTB, PORTB_RB0_POSITION
    CALL DELAY
    BCF PORTB, PORTB_RB0_POSITION

    MOVF DISPLAY_D, W
    MOVWF PORTC
    BSF PORTB, PORTB_RB1_POSITION
    CALL DELAY
    BCF PORTB, PORTB_RB1_POSITION

    RETLW 0


DELAY:
    MOVLW 255
    MOVWF COUNTER1
    MOVLW 128
    MOVWF COUNTER2
DELAY_LOOP:
    DECFSZ COUNTER1, F
    GOTO DELAY_LOOP
    DECFSZ COUNTER2, F
    GOTO DELAY_LOOP
    RETLW 0

;iI2C subroutines--------------------------------------------------------------
    
I2C_START:
    CALL SCL_HIGH
    CALL SDA_LOW
    RETLW 0

I2C_STOP:
    CALL SDA_LOW
    CALL SCL_HIGH
    CALL SDA_HIGH
    RETLW 0
    
SDA_HIGH:
    BSF PORT, SDA
    MOVF PORT, W
    TRIS PORTB
    RETLW 0
    
SDA_LOW:
    BCF PORT, SDA
    MOVF PORT,W
    TRIS PORTB
    RETLW 0
    
SCL_HIGH:
    BSF PORT, SCL
    MOVF PORT, W
    TRIS PORTB
    RETLW 0
    
SCL_LOW:    
    BCF PORT, SCL
    MOVF PORT, W 
    TRIS PORTB
    RETLW 0
    
I2C_WRITE_BYTE:
    MOVWF I2C_DATA
    MOVLW 8
    MOVWF BIT_COUNT
I2C_WRITE_BIT:
    CALL SCL_LOW
    BTFSS I2C_DATA, 7
    GOTO I2C_WRITE_0
I2C_WRITE_1:
    CALL SDA_HIGH
    GOTO I2C_SHIFT
I2C_WRITE_0:
    CALL SDA_LOW
I2C_SHIFT:
    CALL SCL_HIGH
    RLF I2C_DATA, F
    DECFSZ BIT_COUNT, F
    GOTO I2C_WRITE_BIT
I2C_CHECK_ACK:
    CALL SCL_LOW
    CALL SDA_HIGH
    CALL SCL_HIGH
    MOVF PORTB, W
    MOVF ACK
    CALL SCL_LOW
    RETLW 0
    
I2C_READ_BYTE:
    MOVLW 8
    MOVWF BIT_COUNT
    CLRF I2C_DATA
I2C_READ_BIT:
    RLF I2C_DATA, F
    CALL SCL_LOW
    CALL SCL_HIGH
    BTFSC PORTB, SDA
    BSF I2C_DATA, 0
    DECFSZ BIT_COUNT, F
    GOTO I2C_READ_BIT
    CALL SCL_LOW
    RETLW 0
    
I2C_ACK:
    CALL SDA_LOW
    CALL SCL_HIGH
    CALL SCL_LOW
    CALL SDA_HIGH
    RETLW 0
    
I2C_NACK:
    CALL SDA_HIGH
    CALL SCL_HIGH
    CALL SCL_LOW
    RETLW 0
    
    
    
    

