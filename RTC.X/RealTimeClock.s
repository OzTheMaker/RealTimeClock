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
;		  This is the code for the real time clock proyect presented in the OzTheMaker 
;         social media. 
;		  		 
;  
;    Pin assignment:
;
;		        	____________
;		           |            |
;		   VDD >---|	        |---< VSS
;	       DP3 <---|            |---> DP1
;	       DP4 <---|            |---> DP2
;	       SW1 >---|            |---< SW2
;          SCL >--<|            |---> A
;          SDA >--<|            |---> B
;            D <---|            |---> C
;                  |____________|
;
;______________________________________________________________________________;

PROCESSOR 16F506
#include <xc.inc>

;CONFIG word

CONFIG WDTE = OFF
CONFIG CP = OFF
CONFIG MCLRE = ON
CONFIG FOSC = 100b ;Internal clock configured with port RB4 function.

;Program memory reset vector
PSECT resetOrigin, class=CODE, delta=2

;Definitions not included in the device pic-asm specific definitions.

;-----OPTION_REG Bits-----------------------------------------------------------

;-----STATUS Bits---------------------------------------------------------------
Z	EQU 0002h

;Variable definitions
DISPLAY_A EQU 0x0D
DISPLAY_B EQU 0x0E
DISPLAY_C EQU 0x0F
DISPLAY_D EQU 0x10


INIT:
    MOVLW ~((1 << PORTB_RB0_POSITION) | (1 << PORTB_RB1_POSITION) | (1 << PORTB_RB4_POSITION) | (1 << PORTB_RB5_POSITION))
    TRIS PORTB
    MOVLW 0x00
    MOVWF GPIO
    MOVLW ~((1 << PORTC_RC0_POSITION) | (1 << PORTC_RC1_POSITION) | (1 << PORTC_RC2_POSITION) | (1 << PORTC_RC3_POSITION))
    TRIS PORTC

START:
    MOVLW 0X0F
    MOVWF DISPLAY_A
    MOVWF DISPLAY_B
    MOVWF DISPLAY_C
    MOVWF DISPLAY_D

MAIN:
    CALL DISPLAY
    ;create routine to poll SW2
    CALL 






DISPLAY:
MOVF DISPLAY_A, W
MOVWF PORTC
BSF PORTB PORTB_RB0_POSITION
CALL DELAY
BCF PORTB PORTB_RB0_POSITION

MOVF DISPLAY_B, W
MOVWF PORTC
BSF PORTB PORTB_RB1_POSITION
CALL DELAY
BCF PORTB PORTB_RB1_POSITION

MOVF DISPLAY_C, W
MOWF PORTC
BSF PORTB PORTB_RB4_POSITION
CALL DELAY
BCF PORTB PORTB_RB4_POSITION

MOVF DISPLAY_D, W
MOVWF PORTC
BSF PORTB PORTB_RB5_POSITION
CALL DELAY
BCF PORTB PORTB_RB5_POSITION

RETLW 0


DELAY:

DELAY_LOOP:




