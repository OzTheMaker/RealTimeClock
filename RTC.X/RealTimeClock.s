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
;CONFIG word

CONFIG WDT = OFF
CONFIG CP = OFF
CONFIG MCLRE = ON
CONFIG OSC = 100B ;Internal clock configured with port RB4 function.
CONFIG IOSCFS = 0
;Program memory reset vector
PSECT resetOrigin, class=CODE, delta=2
    



INIT:
    MOVLW ~((1 << PORTB_RB0_POSITION) | (1 << PORTB_RB1_POSITION) | (1 << PORTB_RB4_POSITION) | (1 << PORTB_RB5_POSITION))
    TRIS PORTB
    MOVLW 0x00
    MOVWF PORTB
    MOVLW ~((1 << PORTC_RC0_POSITION) | (1 << PORTC_RC1_POSITION) | (1 << PORTC_RC2_POSITION) | (1 << PORTC_RC3_POSITION))
    TRIS PORTC

START:
    MOVLW 0X08
    MOVWF DISPLAY_A
    MOVWF DISPLAY_B
    MOVWF DISPLAY_C
    MOVWF DISPLAY_D

MAIN:
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

DELAY_LOOP:




