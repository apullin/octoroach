
/**********************************************************************
 * � 2006 Microchip Technology Inc.
 *
 * FileName:        traps.c
 * Dependencies:    p33FJ256GP506.h
 * Processor:       dsPIC33F
 * Compiler:        MPLAB� C30 v2.01 or higher
 *
 * SOFTWARE LICENSE AGREEMENT:
 * Microchip Technology Inc. (�Microchip�) licenses this software to you
 * solely for use with Microchip dsPIC� digital signal controller
 * products. The software is owned by Microchip and is protected under
 * applicable copyright laws.  All rights reserved.
 *
 * SOFTWARE IS PROVIDED �AS IS.�  MICROCHIP EXPRESSLY DISCLAIMS ANY
 * WARRANTY OF ANY KIND, WHETHER EXPRESS OR IMPLIED, INCLUDING BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, OR NON-INFRINGEMENT. IN NO EVENT SHALL MICROCHIP
 * BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR CONSEQUENTIAL
 * DAMAGES, LOST PROFITS OR LOST DATA, HARM TO YOUR EQUIPMENT, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
 * ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER SIMILAR COSTS.
 *
 * REVISION HISTORY:
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author            Date      Comments on this revision
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Hrushikesh Vasuki 05/03/05  First release of source file
 * Richard Fischer   07/14/05  Add DMAC exception handler
 * Priyabrata Sinha  01/27/06  Ported to non-prototype devices
 *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * ADDITIONAL NOTES:
 * 1. This file contains trap service routines (handlers) for hardware
 *    exceptions generated by the dsPIC30F device.
 * 2. All trap service routines in this file simply ensure that device
 *    continuously executes code within the trap service routine. Users
 *    may modify the basic framework provided here to suit to the needs
 *    of their application.
 *
 **********************************************************************/

#include "p33Fxxxx.h"
#include "delay.h"
#include "utils.h"
#include "estop.h"

unsigned int k;
void (*getErrLoc(void))(void); // Get Address Error Loc
void (*errLoc)(void); // Function Pointer


void __attribute__((__interrupt__, no_auto_psv)) _OscillatorFail(void);
void __attribute__((__interrupt__, no_auto_psv)) _AddressError(void);
void __attribute__((__interrupt__, no_auto_psv)) _StackError(void);
void __attribute__((__interrupt__, no_auto_psv)) _MathError(void);
void __attribute__((__interrupt__, no_auto_psv)) _DMACError(void);
void __attribute__((__interrupt__, no_auto_psv)) _AltOscillatorFail(void);
void __attribute__((__interrupt__, no_auto_psv)) _AltAddressError(void);
void __attribute__((__interrupt__, no_auto_psv)) _AltStackError(void);
void __attribute__((__interrupt__, no_auto_psv)) _AltMathError(void);
void __attribute__((__interrupt__, no_auto_psv)) _AltDMACError(void);

/* Primary Exception Vector handlers:
   These routines are used if INTCON2bits.ALTIVT = 0.
   All trap service routines in this file simply ensure that device
   continuously executes code within the trap service routine. Users
   may modify the basic framework provided here to suit to the needs
   of their application. */

void __attribute__((__interrupt__, no_auto_psv)) _OscillatorFail(void) {
    INTCON1bits.OSCFAIL = 0; //Clear the trap flag
    while (1);
}

void __attribute__((__interrupt__, no_auto_psv)) _AddressError(void) {
    EmergencyStop();
    LED_1 = 0;
    LED_2 = 0;
    LED_3 = 0;
    INTCON1bits.ADDRERR = 0; //Clear the trap flags
    while (1) {
        //asm volatile("btg   PORTF, #1");
        LED_1 ^= 1;
        delay_ms(100);
        LED_2 ^= 1;
        delay_ms(100);
        LED_3 ^= 1;
        delay_ms(100);
        //for (k=0; k<100; k++) { delay_ms(5); }   // Waste approximatelly 50ms
    };
}

void __attribute__((__interrupt__, no_auto_psv)) _StackError(void) {
    INTCON1bits.STKERR = 0; //Clear the trap flag
    while (1);
}

void __attribute__((__interrupt__, no_auto_psv)) _MathError(void) {
    INTCON1bits.MATHERR = 0; //Clear the trap flag
    while (1);
}

void __attribute__((__interrupt__, no_auto_psv)) _DMACError(void) {
    /* reset status bits, real app should check which ones */
    DMACS0 = 0;

    INTCON1bits.DMACERR = 0; //Clear the trap flag
    while (1);
}

/* Alternate Exception Vector handlers:
   These routines are used if INTCON2bits.ALTIVT = 1.
   All trap service routines in this file simply ensure that device
   continuously executes code within the trap service routine. Users
   may modify the basic framework provided here to suit to the needs
   of their application. */

void __attribute__((__interrupt__, no_auto_psv)) _AltOscillatorFail(void) {
    INTCON1bits.OSCFAIL = 0;
    while (1);
}

void __attribute__((__interrupt__, no_auto_psv)) _AltAddressError(void) {
    INTCON1bits.ADDRERR = 0;
    while (1);
}

void __attribute__((__interrupt__, no_auto_psv)) _AltStackError(void) {
    INTCON1bits.STKERR = 0;
    while (1);
}

void __attribute__((__interrupt__, no_auto_psv)) _AltMathError(void) {
    INTCON1bits.MATHERR = 0;
    while (1);
}

void __attribute__((__interrupt__)) _AltDMACError(void) {
    /* reset status bits, real app should check which ones */
    DMACS0 = 0;

    INTCON1bits.DMACERR = 0; //Clear the trap flag
    while (1);
}
