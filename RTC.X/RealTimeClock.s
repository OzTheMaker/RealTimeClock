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
    CLRF TIMER_COUNTER
    RETLW 0
    
INC_DATA_A:
    INCF DATA_A, F
    MOVLW 0x82
    MOVWF COUNTER2    
    CALL DELAY_LOOP
    MOVLW 0x03
    SUBWF DATA_A, W                  ;DATA_A - 3
    BTFSC STATUS, STATUS_C_POSITION  ;IF DATA_A > 3
    CLRF DATA_A			     ;THEN make DATA_A = 0
    RETLW 0			     ;ELSE return
    
INC_DATA_B:
    INCF DATA_B, F
    MOVLW 0x82
    MOVWF COUNTER2    
    CALL DELAY_LOOP
    MOVLW 0X0A
    SUBWF DATA_B, W                  ;DATA_B - 10
    BTFSC STATUS, STATUS_C_POSITION  ;IF DATA_B > 10
    CLRF DATA_B			     ;THEN make DATA_B = 0
    RETLW 0			     ;ELSE return
  
INC_DATA_C:
    INCF DATA_C, F
    MOVLW 0x82
    MOVWF COUNTER2    
    CALL DELAY_LOOP
    MOVLW 0x07
    SUBWF DATA_C, W                  ;DATA_C - 7
    BTFSC STATUS, STATUS_C_POSITION  ;IF DATA_C > 7
    CLRF DATA_C			     ;THEN make DATA_C = 0
    RETLW 0			     ;ELSE return
    
INC_DATA_D:
    INCF DATA_D, F
    MOVLW 0x82
    MOVWF COUNTER2    
    CALL DELAY_LOOP
    MOVLW 0X0A
    SUBWF DATA_D, W                  ;DATA_D - 10
    BTFSC STATUS, STATUS_C_POSITION  ;IF DATA_D > 10
    CLRF DATA_D			     ;THEN make DATA_D = 0
    RETLW 0			     ;ELSE return
    
GO_SLEEP:
    SLEEP
    
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
    
;Start clock clearing CH bit and set seconds to 0 and set year------------------   
CLOCK_INIT:
    CALL I2C_START
    MOVLW W_ADDRESS
    CALL I2C_WRITE_BYTE
    MOVLW 0x00
    CALL I2C_WRITE_BYTE
    MOVLW 0x00
    CALL I2C_WRITE_BYTE
    CALL I2C_STOP
    CALL DELAY  
    

;Test all displays for three seconds

START:    
    MOVLW 0xC8
    MOVWF TIMER_FLAG	    ;Sets DELAY_FLAG to 240
    CLRF TIMER_COUNTER      ;Clean TIMER_COUNTER
    CLRF TMR0		    ;Clean TMR0
    MOVLW 0x00              ;Load 0 to all the displays
    MOVWF DATA_A
    MOVWF DATA_B
    MOVWF DATA_C
    MOVWF DATA_D
TEST_DISPLAY:
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
    MOVLW 0x10 
    SUBWF TIMER_COUNTER, W		;TIMER_COUNTER - 
    BTFSC STATUS, STATUS_Z_POSITION     ;IF TIMER_COUNTER reaches 45
    GOTO DISPLAY_OFF 			;THEN exit the loop
    CLRF TMR0
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
LOAD_MIN:		    ;Load minutes into displays variables
    MOVF MIN, W
    ANDLW 0x0F
    MOVWF DATA_D
    MOVF MIN, W
    ANDLW 0x70
    CALL ROUTATE
    MOVF COPY, W
    MOVWF DATA_C
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
LOAD_HOUR:		   ;Load hour values into display variables
    MOVF HOUR, W
    ANDLW 0x0F
    MOVWF DATA_B
    MOVF HOUR, W
    ANDLW 0x30
    CALL ROUTATE
    MOVF COPY, W
    MOVWF DATA_A    

CLRF TIMER_COUNTER      ;Clean TIMER_COUNTER
CLRF TMR0		;Clean TMR0    

;Main loop
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
    MOVLW 0x5A 
    SUBWF TIMER_COUNTER, W		;TIMER_COUNTER - 90
    BTFSC STATUS, STATUS_Z_POSITION     ;IF TIMER_COUNTER reaches 90
    GOTO GO_SLEEP 			;THEN sleep
    CLRF TMR0
    GOTO DISPLAY_TIME			;ELSE continue the loop
   

READ_DATE:
    MOVLW 0x82				;SW2 Debounce
    MOVWF COUNTER2
    CALL DELAY_LOOP
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
    ANDLW 0x30
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
    ANDLW 0x10
    CALL ROUTATE
    MOVF COPY, W
    MOVWF DATA_C
DISPLAY_DATE:
    CALL DISPLAY_A
    CALL DISPLAY_B
    CALL DISPLAY_C
    CALL DISPLAY_D  
    BTFSC PORTB, SW2                    ;Polling SW2 
    GOTO $+0x0B				;IF pushed exit loop
    MOVF TMR0, W
    SUBWF TIMER_FLAG, W			;TIMER_FLAG - TMR0
    BTFSS STATUS, STATUS_C_POSITION     ;IF TMR0 > DELAY_FLAG
    GOTO DISDATE_LOOP			;THEN increase TIMER_COUNTER 
    GOTO DISPLAY_DATE			;ELSE continue the loop
DISDATE_LOOP:
    INCF TIMER_COUNTER
    MOVLW 0x5A 
    SUBWF TIMER_COUNTER, W		;TIMER_COUNTER - 90
    BTFSC STATUS, STATUS_Z_POSITION     ;IF TIMER_COUNTER reaches 90 (4sec)
    GOTO GO_SLEEP 			;THEN sleep
    GOTO DISPLAY_DATE			;ELSE continue the loop   
    
    CLRF TIMER_COUNTER
    CLRF TMR0
POLLING_SW2:
    CALL DISPLAY_A
    CALL DISPLAY_B
    CALL DISPLAY_C
    CALL DISPLAY_D  
    BTFSS PORTB, SW2                    ;Polling SW2 
    GOTO READ_MIN			;IF open display hour
    MOVF TMR0, W
    SUBWF TIMER_FLAG, W			;TIMER_FLAG - TMR0
    BTFSS STATUS, STATUS_C_POSITION     ;IF TMR0 > DELAY_FLAG
    GOTO SW2_LOOP			;THEN increase TIMER_COUNTER 
    GOTO DISPLAY_DATE			;ELSE continue the loop
SW2_LOOP:
    INCF TIMER_COUNTER
    MOVLW 0x2D 
    SUBWF TIMER_COUNTER, W		;TIMER_COUNTER - 45
    BTFSC STATUS, STATUS_Z_POSITION     ;IF TIMER_COUNTER reaches 45 (2sec)
    GOTO $+2				;THEN goto exit loop
    GOTO DISPLAY_DATE			;ELSE continue the loop
    
CLRF DATA_A
CLRF DATA_B
CLRF DATA_C
CLRF DATA_D
MOVLW 0x01
MOVWF COPY		;Set COPY BIT0
    
INC_MIN_UNITS:
    CALL DISPLAY_A
    CALL DISPLAY_B
    CALL DISPLAY_C
    BTFSC COPY, 0x00			;IF COPY BIT0 is 1
    CALL DISPLAY_D			;THEN display D value
    BTFSC PORTB, SW1                    ;Polling SW2 
    CALL INC_DATA_D			;IF SW2 is pressed increase DATA_D
    BTFSC PORTB, SW2			;Polling SW1
    GOTO $+0x0D				;IF SW1 is pressed exit loop
    MOVF TMR0, W
    SUBWF TIMER_FLAG, W			;TIMER_FLAG - TMR0
    BTFSS STATUS, STATUS_C_POSITION     ;IF TMR0 > DELAY_FLAG
    GOTO MIN_UNITS_LOOP			;THEN increase TIMER_COUNTER 
    GOTO INC_MIN_UNITS			;ELSE continue the loop
MIN_UNITS_LOOP:
    INCF TIMER_COUNTER
    MOVLW 0x03 
    SUBWF TIMER_COUNTER, W		;TIMER_COUNTER - 16
    BTFSC STATUS, STATUS_Z_POSITION     ;IF TIMER_COUNTER reaches 16
    CALL TOGGLE_COPY			;THEN toggle COPY BIT0
    CLRF TMR0				;Reset timer
    GOTO INC_MIN_UNITS			;ELSE continue the loop
    
MOVLW 0x90
MOVWF COUNTER2    
CALL DELAY_LOOP
CLRF TIMER_COUNTER    
CLRF TMR0
CLRF COPY 
    
INC_MIN_DEC:
    CALL DISPLAY_A
    CALL DISPLAY_B
    BTFSC COPY, 0x00			;IF COPY BIT0 is 1
    CALL DISPLAY_C			;THEN display C value
    CALL DISPLAY_D			
    BTFSC PORTB, SW1                    ;Polling SW2 
    CALL INC_DATA_C			;IF SW2 is pressed increase DATA_C
    BTFSC PORTB, SW2			;Polling SW1
    GOTO $+0x0D				;IF SW1 is pressed exit loop
    MOVF TMR0, W
    SUBWF TIMER_FLAG, W			;TIMER_FLAG - TMR0
    BTFSS STATUS, STATUS_C_POSITION     ;IF TMR0 > DELAY_FLAG
    GOTO MIN_DEC_LOOP			;THEN increase TIMER_COUNTER 
    GOTO INC_MIN_DEC			;ELSE continue the loop
MIN_DEC_LOOP:
    INCF TIMER_COUNTER
    MOVLW 0x03 
    SUBWF TIMER_COUNTER, W		;TIMER_COUNTER - 16
    BTFSC STATUS, STATUS_Z_POSITION     ;IF TIMER_COUNTER reaches 16
    CALL TOGGLE_COPY			;THEN toggle COPY BIT0
    CLRF TMR0				;Reset timer
    GOTO INC_MIN_DEC			;ELSE continue the loop

MOVLW 0x90
MOVWF COUNTER2    
CALL DELAY_LOOP     
CLRF TIMER_COUNTER      
CLRF TMR0
CLRF COPY 
    
INC_HOUR_UNITS:
    CALL DISPLAY_A
    BTFSC COPY, 0x00			;IF COPY BIT0 is 1
    CALL DISPLAY_B			;THEN display B value
    CALL DISPLAY_C			
    CALL DISPLAY_D			
    BTFSC PORTB, SW1                    ;Polling SW2 
    CALL INC_DATA_B			;IF SW2 is pressed increase DATA_B
    BTFSC PORTB, SW2			;Polling SW1
    GOTO $+0x0D				;IF SW1 is pressed exit loop
    MOVF TMR0, W
    SUBWF TIMER_FLAG, W			;TIMER_FLAG - TMR0
    BTFSS STATUS, STATUS_C_POSITION     ;IF TMR0 > DELAY_FLAG
    GOTO HOUR_UNITS_LOOP			;THEN increase TIMER_COUNTER 
    GOTO INC_HOUR_UNITS			;ELSE continue the loop
HOUR_UNITS_LOOP:
    INCF TIMER_COUNTER
    MOVLW 0x03 
    SUBWF TIMER_COUNTER, W		;TIMER_COUNTER - 16
    BTFSC STATUS, STATUS_Z_POSITION     ;IF TIMER_COUNTER reaches 16
    CALL TOGGLE_COPY			;THEN toggle COPY BIT0
    CLRF TMR0				;Reset timer
    GOTO INC_HOUR_UNITS			;ELSE continue the loop

MOVLW 0x90
MOVWF COUNTER2    
CALL DELAY_LOOP    
CLRF TIMER_COUNTER      
CLRF TMR0
CLRF COPY    

INC_HOUR_DEC:
    BTFSC COPY, 0x00			;IF COPY BIT0 is 1
    CALL DISPLAY_A			;THEN display A value
    CALL DISPLAY_B			
    CALL DISPLAY_C			
    CALL DISPLAY_D			
    BTFSC PORTB, SW1                    ;Polling SW2 
    CALL INC_DATA_A			;IF SW2 is pressed increase DATA_A
    BTFSC PORTB, SW2			;Polling SW1
    GOTO $+0x0D				;IF SW1 is pressed exit loop
    MOVF TMR0, W
    SUBWF TIMER_FLAG, W			;TIMER_FLAG - TMR0
    BTFSS STATUS, STATUS_C_POSITION     ;IF TMR0 > DELAY_FLAG
    GOTO HOUR_DEC_LOOP			;THEN increase TIMER_COUNTER 
    GOTO INC_HOUR_DEC			;ELSE continue the loop
HOUR_DEC_LOOP:
    INCF TIMER_COUNTER
    MOVLW 0x03 
    SUBWF TIMER_COUNTER, W		;TIMER_COUNTER - 16
    BTFSC STATUS, STATUS_Z_POSITION     ;IF TIMER_COUNTER reaches 16
    CALL TOGGLE_COPY			;THEN toggle COPY BIT0
    CLRF TMR0				;Reset timer
    GOTO INC_HOUR_DEC			;ELSE continue the loop

MOVLW 0x90
MOVWF COUNTER2    
CALL DELAY_LOOP
CLRF TIMER_COUNTER      
CLRF TMR0
CLRF COPY 
    
SEND_TIME:
    ;Adding minutes
    RLF DATA_C
    RLF DATA_C
    RLF DATA_C
    RLF DATA_C
    MOVF DATA_C, W
    ANDLW 0xF0
    MOVWF DATA_C
    MOVF DATA_D, W
    ANDLW 0x0F
    MOVWF DATA_D
    MOVF DATA_D, W			
    ADDWF DATA_C, W			;DATA_D + DATA_C
    MOVWF COPY
    CALL I2C_START
    MOVLW W_ADDRESS
    CALL I2C_WRITE_BYTE
    MOVLW 0X01				
    CALL I2C_WRITE_BYTE			;Writes over minute address
    
    RLF DATA_A
    RLF DATA_A
    RLF DATA_A
    RLF DATA_A
    MOVF DATA_A, W
    ANDLW 0x30
    MOVWF DATA_A
    MOVF DATA_B, W
    ANDLW 0x0F
    MOVWF DATA_B
    MOVF DATA_A, W
    ADDWF DATA_B, W
    MOVWF COPY
    CALL I2C_WRITE_BYTE			;Writes over hour address
    CALL I2C_STOP
 
    GOTO CLOCK_INIT
    
END
    
    

