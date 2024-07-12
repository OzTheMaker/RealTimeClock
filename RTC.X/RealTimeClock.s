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

;Timer configuration
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

;Start clock clearing CH bit and set seconds to 0    
CLOCK_INIT:
    CALL I2C_START
    MOVLW W_ADDRESS
    CALL I2C_WRITE_BYTE
    MOVLW 0x00
    CALL I2C_WRITE_BYTE
    MOVLW 0x00
    CALL I2C_WRITE_BYTE
    CALL I2C_STOP

;Test all displays for three seconds

START:    
    MOVLW 0x240
    MOVWF TIMER_FLAG	    ;Sets DELAY_FLAG to 240
    CLRF TIMER_COUNTER      ;Clean TIMER_COUNTER
    CLRF TMR0		    ;Clean TMR0
    
TEST_DISPLAY:
    MOVLW 0x08  ;Load 8 to all the displays
    MOVWF DATA_A
    MOVWF DATA_B
    MOVWF DATA_C
    MOVWF DATA_D
    CALL DISPLAY_A
    CALL DISPLAY_B
    CALL DISPLAY_C
    CALL DISPLAY_D
    MOVF TMR0, W
    SUBWF TIMER_FLAG, W			;TIMER_FLAG - TMR0
    BTFSS STATUS, STATUS_C_POSITION     ;IF TMR0 > DELAY_FLAG
    GOTO TEST_LOOP			;THEN increase TIMER_COUNTER 
    GOTO TEST_DISPLAY			;ELSE continue the loop
TEST_LOOP:
    INCF TIMER_COUNTER
    MOVLW 0x2D 
    SUBWF TIMER_COUNTER, W		;TIMER_COUNTER - 45
    BTFSC STATUS, STATUS_Z_POSITION     ;IF TIMER_COUNTER reaches 45
    GOTO DISPLAY_OFF 			;THEN exit the loop
    GOTO TEST_DISPLAY			;ELSE continue the loop

DISPLAY_OFF:
    CLRF PORTB

;Read and load time values
READ_MIN:
    CALL I2C_START
    MOVLW W_ADDRESS
    CALL I2C_WRITE_BYTE
    MOVLW 0x01
    CALL I2C_WRITE_BYTE
    CALL I2C_START
    MOVLW R_ADDRESS
    CALL I2C_WRITE_BYTE
    CALL I2C_READ_BYTE
    CALL I2C_NACK
    MOVF I2C_DATA, W
    MOVWF MIN
    CALL I2C_STOP
READ_HOUR:
    CALL I2C_START
    MOVLW W_ADDRESS
    CALL I2C_WRITE_BYTE
    MOVLW 0x02
    CALL I2C_WRITE_BYTE
    CALL I2C_START
    MOVLW R_ADDRESS
    CALL I2C_WRITE_BYTE
    CALL I2C_READ_BYTE
    CALL I2C_NACK
    MOVF I2C_DATA, W
    MOVWF HOUR
    CALL I2C_STOP    
LOAD_MIN:		    ;Load minutes in displays DC
    MOVF MIN, W
    ANDLW 0x0F
    MOVWF DATA_D
    MOVF MIN, W
    ANDLW 0xF0
    CALL ROUTATE
    MOVF COPY, W
    MOVWF DATA_C
LOAD_HOUR:		   ;Load hour in displays BA
    MOVF HOUR, W
    ANDLW 0x0F
    MOVWF DATA_B
    MOVF HOUR, W
    ANDLW 0x10
    CALL ROUTATE
    MOVF COPY, W
    MOVWF DATA_A    

CLRF TIMER_COUNTER      ;Clean TIMER_COUNTER
CLRF TMR0		;Clean TMR0    

DISPLAY_TIME:
    CALL DISPLAY_A
    CALL DISPLAY_B
    CALL DISPLAY_C
    CALL DISPLAY_D
    BTFSC PORTB, SW2                    ;Polling SW2 
    GOTO READ_DATE			;IF pushed display date
    MOVF TMR0, W
    SUBWF TIMER_FLAG, W			;TIMER_FLAG - TMR0
    BTFSS STATUS, STATUS_C_POSITION     ;IF TMR0 > DELAY_FLAG
    GOTO DISTIME_LOOP			;THEN increase TIMER_COUNTER 
    GOTO DISPLAY_TIME			;ELSE continue the loop
DISTIME_LOOP:
    INCF TIMER_COUNTER
    MOVLW 0x2D 
    SUBWF TIMER_COUNTER, W		;TIMER_COUNTER - 45
    BTFSC STATUS, STATUS_Z_POSITION     ;IF TIMER_COUNTER reaches 45
    GOTO GO_SLEEP 			;THEN exit the loop
    GOTO DISPLAY_TIME			;ELSE continue the loop
   

READ_DATE:
    CLRF PORTB				;Turn off display
    CLRF TMR0
    CLRF TIMER_COUNTER
    CALL I2C_START
    MOVLW W_ADDRESS
    CALL I2C_WRITE_BYTE
    MOVLW 0x04
    CALL I2C_WRITE_BYTE
    CALL I2C_START
    MOVLW R_ADDRESS
    CALL I2C_WRITE_BYTE
    CALL I2C_READ_BYTE
    CALL I2C_NACK
    MOVF I2C_DATA, W
    MOVWF DATE
    CALL I2C_STOP
LOAD_DATE:    
    MOVF DATE, W
    ANDLW 0x0F
    MOVWF DATA_B
    MOVF DATE, W
    ANDLW 0xF0
    CALL ROUTATE
    MOVF COPY, W
    MOVWF DATA_A
READ_MONTH:
    CALL I2C_START
    MOVLW W_ADDRESS
    CALL I2C_WRITE_BYTE
    MOVLW 0x05
    CALL I2C_WRITE_BYTE
    CALL I2C_START
    MOVLW R_ADDRESS
    CALL I2C_WRITE_BYTE
    CALL I2C_READ_BYTE
    CALL I2C_NACK
    MOVF I2C_DATA, W
    MOVWF MONTH
    CALL I2C_STOP
LOAD_MONTH:
    MOVF MONTH, W
    ANDLW 0x0F
    MOVWF DATA_D
    MOVF MONTH, W
    ANDLW 0xF0
    CALL ROUTATE
    MOVF COPY, W
    MOVWF DATA_C
DISPLAY_DATE:
    CALL DISPLAY_A
    CALL DISPLAY_B
    CALL DISPLAY_C
    CALL DISPLAY_D    
    BTFSC PORTB, SW2                    ;Polling SW2 
    GOTO UPDATE_RTC			;IF pushed goto UPDATE_RTC
    MOVF TMR0, W
    SUBWF TIMER_FLAG, W			;TIMER_FLAG - TMR0
    BTFSS STATUS, STATUS_C_POSITION     ;IF TMR0 > DELAY_FLAG
    GOTO DISDATE_LOOP			;THEN increase TIMER_COUNTER 
    GOTO DISPLAY_DATE			;ELSE continue the loop
DISDATE_LOOP:
    INCF TIMER_COUNTER
    MOVLW 0x2D 
    SUBWF TIMER_COUNTER, W		;TIMER_COUNTER - 45
    BTFSC STATUS, STATUS_Z_POSITION     ;IF TIMER_COUNTER reaches 45
    GOTO GO_SLEEP 			        ;THEN exit the loop
    GOTO DISPLAY_DATE			;ELSE continue the loop   
    
UPDATE_RTC:
    CLRF TIMER_COUNTER
    CLRF TMR0
    CALL DISPLAY_A
    CALL DISPLAY_B
    CALL DISPLAY_C
    CALL DISPLAY_D
    
GO_SLEEP:
    GOTO START     
;create routine to poll SW2
    

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
    
DELAY:
    MOVLW 0x128
    MOVWF COUNTER1
    MOVLW 0X10
    MOVWF COUNTER2
DELAY_LOOP:
    DECFSZ COUNTER1, F
    GOTO DELAY_LOOP
    DECFSZ COUNTER2, F
    GOTO DELAY_LOOP
    RETLW 0
    
ROUTATE:
    MOVWF COPY
    RRF COPY, F
    RRF COPY, F
    RRF COPY, F
    RRF COPY, F
    RETLW 0
    
READ_DAY:
    CALL I2C_START
    MOVLW W_ADDRESS
    CALL I2C_WRITE_BYTE
    MOVLW 0x03
    CALL I2C_WRITE_BYTE
    CALL I2C_START
    MOVLW R_ADDRESS
    CALL I2C_WRITE_BYTE
    CALL I2C_READ_BYTE
    CALL I2C_NACK
    MOVF I2C_DATA, W
    MOVWF DAY
    CALL I2C_STOP
    
    


READ_YEAR:
    CALL I2C_START
    MOVLW W_ADDRESS
    CALL I2C_WRITE_BYTE
    MOVLW 0x06
    CALL I2C_WRITE_BYTE
    CALL I2C_START
    MOVLW R_ADDRESS
    CALL I2C_WRITE_BYTE
    CALL I2C_READ_BYTE
    CALL I2C_NACK
    MOVF I2C_DATA, W
    MOVWF YEAR
    CALL I2C_STOP


END
    
    

