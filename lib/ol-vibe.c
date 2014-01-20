/*
 * File:   ol-vibe.c
 * Author: pullin
 *
 * Created on January 20, 2014, 1:59 PM
 */

#include <xc.h>

#include "ol-vibe.h"
#include "tih.h"
//#include "math.h"  //for abs
#include "timer.h"

#define ABS(my_val) ((my_val) < 0) ? -(my_val) : (my_val)

#define T3FREQ 5000
#define MAXPWM 1999

//static unsigned int maxDC; // from define, MAXPWM
static unsigned int olvfreq;
static int chan1amp;
static int chan2amp;

//static unsigned char running = 0;

static unsigned char olphase = 0; // 0 = in phase, 1 = antiphase

//Private functions
static void SetupTimer3(void);
static void SetupOC1(void);

void olVibeSetup() {
    //get max DC from PWM or leg ctrl module

    //Set up timer
    SetupTimer3();
    //Set up output compare peripheral
    SetupOC1();
    //Default to 5hz @ 50% maximum
    olVibeSetAmplitude(1, MAXPWM / 2);
    olVibeSetAmplitude(2, MAXPWM / 2);
    olVibeSetFrequency(5);
}

void olVibeStart(void) {
    _OC1IE = 1;
}

void olVibeStop(void) {
    _OC1IE = 0;
}

void olVibeSetFrequency(unsigned int freq) {
    T3CONbits.TON = 0; //stop timer

    olvfreq = freq;
    PR3 = 8000 / freq;
    OC1R = PR3 >> 1;

    T3CONbits.TON = 1; //start timer
}

void olVibeSetAmplitude(unsigned int channel, unsigned int amp) {
    if (channel == 1) {
        chan1amp = amp;
    } else if (channel == 2) {
        chan2amp = amp;
    }

}

void olVibeSetAmplitudeFloat(unsigned int channel, float famp) {

}

void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void) {
    chan1amp = -chan1amp; //toggle direction
    tiHSetDC(1, chan1amp);
    chan2amp = -chan2amp; //toggle direction
    tiHSetDC(2, chan2amp);

    IFS0bits.OC1IF = 0; // Clear OC1 interrupt flag
}

void olVibeSetPhase(unsigned char phase) {
    //running = 0;
    //Don't need to do anything if frequencies are not the same.
    //Dissimilar frequencies will naturally cross.

    olphase = phase; //store locally

    _OC1IE = 0;
    if (phase == 0) {
        chan1amp = ABS(chan1amp);
        chan2amp = ABS(chan2amp);
    } else //antiphase
    {
        chan1amp = ABS(chan1amp);
        chan2amp = -ABS(chan2amp);
    }

    //running = 1;
    _OC1IE = 1;
}

//Private funcitons

void SetupTimer3() {
    unsigned int T3CON1value, T3PERvalue;
    T3CON1value = T3_ON & T3_SOURCE_INT & T3_PS_1_1 & T3_GATE_OFF &
            T1_SYNC_EXT_OFF & T3_INT_PRIOR_2;
    T3PERvalue = 8000; //clock period = 0.0002s = ((T1PERvalue * prescaler)/FCY) (5000Hz)
    OpenTimer3(T3CON1value, T3PERvalue);
}

static void SetupOC1(void) {
    // Initialize Output Compare Module
    OC1CONbits.OCM = 0b000; // Disable Output Compare Module
    OC1CONbits.OCM = 0b010; // Define Initial State for OC1 Pin (High if OCM=0b010)
    OC1CONbits.OCM = 0b000; // Disable Output Compare Module
    OC1CONbits.OCTSEL = 1; // Select Timer3 as output compare time base
    OC1R = PR3 >> 1; // Toggle mode, value should be irrelevant? Should always be 50% timer period
    IPC0bits.OC1IP = 0x01; // Set Output Compare 1 Interrupt Priority Level, TODO: probably wrong! AP 1/20/14
    IFS0bits.OC1IF = 0; // Clear Output Compare 1 Interrupt Flag
    //IEC0bits.OC1IE = 1; // Enable Output Compare 1 interrupt
    OC1CONbits.OCM = 0b011; // Select the Output Compare mode
}