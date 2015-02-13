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
#include "libq.h"
#include <stdint.h>

#define ABS(my_val) ((my_val) < 0) ? -(my_val) : (my_val)

#define T3FREQ 2048
#define MAXPWM 1999 //20khz pwm
//#define MAXPWM 0xf80 //10khz pwm

//static unsigned int maxDC; // from define, MAXPWM
//static unsigned int olvfreq;
static int chan1amp, chan2amp;
static int chan1dc, chan2dc;
static _Q15 chan1arg, chan2arg;
static _Q15 chan1_delta, chan2_delta;

static unsigned char running = 0;

//static unsigned char olphase = 0; // 0 = in phase, 1 = antiphase

//Private functions
static void SetupTimer3(void);
//static void SetupOC1(void);

void olVibeSetup() {
    //get max DC from PWM or leg ctrl module

    //Set up timer
    SetupTimer3();
    //Set up output compare peripheral
    //SetupOC1();
    //Default to 50hz @ 0% drive
    olVibeSetAmplitude(1, 0);
    olVibeSetAmplitude(2, 0);
    olVibeSetFrequency(1,2621); //50 hz
    olVibeSetFrequency(2,2621); //50 hz
    
    olVibeStart();
}

void olVibeStart(void) {
    //_OC1IE = 1;
    TMR3 = 0; //reset timer counter, so sin arg starts at 0
    T3CONbits.TON = 1;
    _T3IE = 1;
}

void olVibeStop(void) {
    //_OC1IE = 0;
    T3CONbits.TON = 0;
}

void olVibeSetFrequency(unsigned int channel, unsigned int incr) {

     if (channel == 1) {
        chan1_delta = incr;
    } else if (channel == 2) {
        chan2_delta = incr;
    }

}

void olVibeSetAmplitude(unsigned int channel, unsigned int amp) {
    if (channel == 1) {
        chan1amp = amp;
    } else if (channel == 2) {
        chan2amp = amp;
    }

    if(amp != 0){
        running = 1;
    }
    else{
        running = 0;
    }

}

void olVibeSetAmplitudeFloat(unsigned int channel, float famp) {
    //unimplemented
}

/*
void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void) {
    chan1amp = -chan1amp; //toggle direction
    tiHSetDC(1, chan1amp);
    chan2amp = -chan2amp; //toggle direction
    tiHSetDC(2, chan2amp);

    IFS0bits.OC1IF = 0; // Clear OC1 interrupt flag
}
*/

void olVibeSetPhase(unsigned int channel, _Q15 phase) {
    //phase should be a _Q15, correpsonding to [-pi, pi]

    if (channel == 1) {
        chan1arg += phase;
    } else if (channel == 2) {
        chan2arg += phase;
    }

}

//Private funcitons

void SetupTimer3() {
    unsigned int T3CON1value, T3PERvalue;
    T3CON1value = T3_ON & T3_SOURCE_INT & T3_PS_1_8 & T3_GATE_OFF &
            T1_SYNC_EXT_OFF & T3_INT_PRIOR_6;
    T3PERvalue = 4000; //1250 Hz for 1:8 divider
    OpenTimer3(T3CON1value, T3PERvalue);
    _T3IE = 0;
}

/*
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
}*/


void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void) {
    //chan1amp = -chan1amp; //toggle direction
    //tiHSetDC(1, chan1amp);

    if(running){
        Nop();
        chan1arg += chan1_delta;
        chan2arg += chan2_delta;

        chan1dc = _Q15sinPI(chan1arg);
        long temp = (long)(chan1dc) * (long)chan1amp;
        chan1dc = (int)(temp >> 15);

        chan2dc = _Q15sinPI(chan2arg);
        temp = (long)(chan2dc) * (long)chan2amp;
        chan2dc = (int)(temp >> 15);

        tiHSetDC(1, chan1dc);
        tiHSetDC(2, chan2dc);
        Nop();
    }
    else{
        tiHSetDC(1, 0);
        tiHSetDC(2, 0);
    }
    _T3IF = 0;
}
