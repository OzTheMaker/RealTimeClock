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
DATA_A EQU 0x0D
DATA_B EQU 0x0E
DATA_C EQU 0x0F
DATA_D EQU 0x10
COUNTER1 EQU 0x11
COUNTER2 EQU 0x12 
MIN	EQU 0x13
HOUR	EQU 0x14
DAY	EQU 0x15
MONTH	EQU 0x16
YEAR	EQU 0x17
DATE	EQU 0X18	
SDA EQU 0x04
SCL EQU 0x05
SW1 EQU 0x03
SW2 EQU 0x02 
BIT_COUNT EQU 0x19
I2C_DATA EQU 0x1A 
PORT EQU 0x1B
ACK EQU 0x1C
COPY EQU 0x1D
W_ADDRESS EQU 0xD0   
R_ADDRESS EQU 0xD1
TIMER_COUNTER EQU 0x1E
TIMER_FLAG EQU 0x1F
 
 ;CONFIG word
CONFIG WDT = OFF
CONFIG CP = OFF
CONFIG MCLRE = OFF
CONFIG OSC = 100B ;Internal clock configured with port RB4 function.
CONFIG IOSCFS = 0
 
;Program memory reset vector
PSECT resetOrigin, class=CODE, delta=2
 
GOTO TIMER_INIT
 
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
    TRIS PORTC
    RETLW 0
    
SDA_LOW:
    BCF PORT, SDA
    MOVF PORT,W
    TRIS PORTC
    RETLW 0
    
SCL_HIGH:
    BSF PORT, SCL
    MOVF PORT, W
    TRIS PORTC
    RETLW 0
    
SCL_LOW:    
    BCF PORT, SCL
    MOVF PORT, W 
    TRIS PORTC
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
    MOVF PORTC, W
    MOVWF ACK
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
    BTFSC PORTC, SDA
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

;Helper subroutines------------------------------------------------------------
DELAY:
    MOVLW 0x80
    MOVWF COUNTER1
    MOVLW 0X10
    MOVWF COUNTER2
DELAY_LOOP:
    DECFSZ COUNTER1, F
    GOTO DELAY_LOOP
    DECFSZ COUNTER2, F
    GOTO DELAY_LOOP
    RETLW 0
    
DISPLAY_A:
    MOVF DATA_A, W
    MOVWF PORTC
    BSF PORTB, PORTB_RB5_POSITION
    CALL DELAY
    BCF PORTB, PORTB_RB5_POSITION
    RETLW 0
    
DISPLAY_B:    
    MOVF DATA_B, W
    MOVWF PORTC
    BSF PORTB, PORTB_RB4_POSITION
    CALL DELAY
    BCF PORTB, PORTB_RB4_POSITION
    RETLW 0
    
DISPLAY_C:
    MOVF DATA_C, W
    MOVWF PORTC
    BSF PORTB, PORTB_RB0_POSITION
    CALL DELAY
    BCF PORTB, PORTB_RB0_POSITION
    RETLW 0
    
DISPLAY_D:    
    MOVF DATA_D, W
    MOVWF PORTC
    BSF PORTB, PORTB_RB1_POSITION
    CALL DELAY
    BCF PORTB, PORTB_RB1_POSITION
    RETLW 0
     
ROUTATE:
    BCF STATUS, STATUS_C_POSITION
    MOVWF COPY
    RRF COPY, F
    RRF COPY, F
    RRF COPY, F
    RRF COPY, F
    RETLW 0
    
TOGGLE_COPY:
    MOVLW 0x01
    XORWF COPY, F		    ;Toggle COPY
    RETLW 0
    
INC_DATA_A:
    INCF DATA_A
    CALL DELAY
    MOVLW 0X02
    SUBWF DATA_A, W                  ;DATA_A - 2
    BTFSS STATUS, STATUS_C_POSITION  ;IF DATA_A > 2
    CLRF DATA_A			     ;THEN make DATA_A = 0
    RETLW 0			     ;ELSE return
    
INC_DATA_B:
    INCF DATA_B
    CALL DELAY
    MOVLW 0X09
    SUBWF DATA_B, W                  ;DATA_B - 9
    BTFSS STATUS, STATUS_C_POSITION  ;IF DATA_B > 9
    CLRF DATA_B			     ;THEN make DATA_B = 0
    RETLW 0			     ;ELSE return
  
INC_DATA_C:
    INCF DATA_C
    CALL DELAY
    MOVLW 0X09
    SUBWF DATA_C, W                  ;DATA_C - 9
    BTFSS STATUS, STATUS_C_POSITION  ;IF DATA_C > 9
    CLRF DATA_C			     ;THEN make DATA_C = 0
    RETLW 0			     ;ELSE return
    
INC_DATA_D:
    INCF DATA_D
    CALL DELAY
    MOVLW 0X09
    SUBWF DATA_D, W                  ;DATA_D - 2
    BTFSS STATUS, STATUS_C_POSITION  ;IF DATA_D > 2
    CLRF DATA_D			     ;THEN make DATA_D = 0
    RETLW 0			     ;ELSE return
    
    
;Timer configuration-----------------------------------------------------------
TIMER_INIT:
    CALL DELAY  ;Stabilizes the oscillator
    MOVLW 0xDF  ;Set TMR0 to timer mode
    OPTION
    CLRWDT
    MOVLW 0xD7  ;Switch prescaler to TMR0 and set its value to 256
    OPTION
    
;Setting up MCU pins to work as GPIO    
GPIO_INIT:
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
    CLRF PORTC
    CLRF PORT
    
;Test all displays for three seconds

START:    
    MOVLW 0xC8
    MOVWF TIMER_FLAG	    ;Sets DELAY_FLAG to 240
    CLRF TIMER_COUNTER      ;Clean TIMER_COUNTER
    CLRF TMR0		    ;Clean TMR0
    
DISPLAY_OFF:
    CLRF PORTB
    ;Load sample values to display
    MOVLW 0x01              
    MOVWF DATA_A
    MOVLW 0X02
    MOVWF DATA_B
    MOVLW 0X03
    MOVWF DATA_C
    MOVWF 0X04
    MOVWF DATA_D
    CLRF  COPY		    ;Clean COPY

MODIFY_D:
    CALL DISPLAY_A
    CALL DISPLAY_B
    CALL DISPLAY_C
    BTFSC COPY, 0x00			;IF COPY BIT0 is 1
    CALL DISPLAY_D			;THEN display D value
    BTFSC PORTB, SW2                    ;Polling SW2 
    CALL INC_DATA_D			;IF SW2 is pressed increase DATA_D
    BTFSC PORTB, SW1			;Polling SW1
    GOTO $+0x0D				;IF SW1 is pressed exit loop
    MOVF TMR0, W
    SUBWF TIMER_FLAG, W			;TIMER_FLAG - TMR0
    BTFSS STATUS, STATUS_C_POSITION     ;IF TMR0 > DELAY_FLAG
    GOTO MODIFY_D_LOOP			;THEN increase TIMER_COUNTER 
    GOTO MODIFY_D			;ELSE continue the loop
MODIFY_D_LOOP:
    INCF TIMER_COUNTER
    MOVLW 0x10 
    SUBWF TIMER_COUNTER, W		;TIMER_COUNTER - 16
    BTFSC STATUS, STATUS_Z_POSITION     ;IF TIMER_COUNTER reaches 16
    CALL TOGGLE_COPY			;THEN toggle COPY BIT0
    CLRF TMR0				;Reset timer
    GOTO MODIFY_D			;ELSE continue the loop

CLRF TIMER_COUNTER    
CLRF TMR0
CLRF COPY

MODIFY_C:
    CALL DISPLAY_A
    CALL DISPLAY_B
    BTFSC COPY, 0x00			;IF COPY BIT0 is 1
    CALL DISPLAY_C			;THEN display C value
    CALL DISPLAY_D			
    BTFSC PORTB, SW2                    ;Polling SW2 
    CALL INC_DATA_C			;IF SW2 is pressed increase DATA_C
    BTFSC PORTB, SW1			;Polling SW1
    GOTO $+0x0D				;IF SW1 is pressed exit loop
    MOVF TMR0, W
    SUBWF TIMER_FLAG, W			;TIMER_FLAG - TMR0
    BTFSS STATUS, STATUS_C_POSITION     ;IF TMR0 > DELAY_FLAG
    GOTO MODIFY_C_LOOP			;THEN increase TIMER_COUNTER 
    GOTO MODIFY_C			;ELSE continue the loop
MODIFY_C_LOOP:
    INCF TIMER_COUNTER
    MOVLW 0x10 
    SUBWF TIMER_COUNTER, W		;TIMER_COUNTER - 16
    BTFSC STATUS, STATUS_Z_POSITION     ;IF TIMER_COUNTER reaches 16
    CALL TOGGLE_COPY			;THEN toggle COPY BIT0
    CLRF TMR0				;Reset timer
    GOTO MODIFY_C			;ELSE continue the loop

CLRF TIMER_COUNTER      
CLRF TMR0
CLRF COPY    

MODIFY_B:
    CALL DISPLAY_A
    BTFSC COPY, 0x00			;IF COPY BIT0 is 1
    CALL DISPLAY_B			;THEN display B value
    CALL DISPLAY_C			
    CALL DISPLAY_D			
    BTFSC PORTB, SW2                    ;Polling SW2 
    CALL INC_DATA_B			;IF SW2 is pressed increase DATA_B
    BTFSC PORTB, SW1			;Polling SW1
    GOTO $+0x0D				;IF SW1 is pressed exit loop
    MOVF TMR0, W
    SUBWF TIMER_FLAG, W			;TIMER_FLAG - TMR0
    BTFSS STATUS, STATUS_C_POSITION     ;IF TMR0 > DELAY_FLAG
    GOTO MODIFY_B_LOOP			;THEN increase TIMER_COUNTER 
    GOTO MODIFY_B			;ELSE continue the loop
MODIFY_B_LOOP:
    INCF TIMER_COUNTER
    MOVLW 0x10 
    SUBWF TIMER_COUNTER, W		;TIMER_COUNTER - 16
    BTFSC STATUS, STATUS_Z_POSITION     ;IF TIMER_COUNTER reaches 16
    CALL TOGGLE_COPY			;THEN toggle COPY BIT0
    CLRF TMR0				;Reset timer
    GOTO MODIFY_B			;ELSE continue the loop

CLRF TIMER_COUNTER      
CLRF TMR0
CLRF COPY    
  
MODIFY_A:
    BTFSC COPY, 0x00			;IF COPY BIT0 is 1
    CALL DISPLAY_A			;THEN display A value
    CALL DISPLAY_B			
    CALL DISPLAY_C			
    CALL DISPLAY_D			
    BTFSC PORTB, SW2                    ;Polling SW2 
    CALL INC_DATA_A			;IF SW2 is pressed increase DATA_A
    BTFSC PORTB, SW1			;Polling SW1
    GOTO $+0x0D				;IF SW1 is pressed exit loop
    MOVF TMR0, W
    SUBWF TIMER_FLAG, W			;TIMER_FLAG - TMR0
    BTFSS STATUS, STATUS_C_POSITION     ;IF TMR0 > DELAY_FLAG
    GOTO MODIFY_A_LOOP			;THEN increase TIMER_COUNTER 
    GOTO MODIFY_A			;ELSE continue the loop
MODIFY_A_LOOP:
    INCF TIMER_COUNTER
    MOVLW 0x10 
    SUBWF TIMER_COUNTER, W		;TIMER_COUNTER - 16
    BTFSC STATUS, STATUS_Z_POSITION     ;IF TIMER_COUNTER reaches 16
    CALL TOGGLE_COPY			;THEN toggle COPY BIT0
    CLRF TMR0				;Reset timer
    GOTO MODIFY_A			;ELSE continue the loop

CLRF TIMER_COUNTER      
CLRF TMR0
CLRF COPY 
    
GO_SLEEP:
    GOTO START   
        
END
    
    

