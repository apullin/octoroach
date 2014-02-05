// Contents of this file are copyright Andrew Pullin, 2013

//leg_ctrl.c

#include "pid.h"
#include "leg_ctrl.h"
//#include "motor_ctrl.h"
#include "tih.h"
#include "timer.h"
#include "adc_pid.h"
#include "pwm.h"
#include "adc.h"
#include "move_queue.h"
#include "tail_queue.h"
#include "math.h"
#include "steering.h"
#include "sys_service.h"
#include "utils.h"
#include "settings.h"
#include <dsp.h>
#include <stdlib.h> // for malloc
#include <limits.h> //for INT_MAX, etc

#define ABS(my_val) ((my_val) < 0) ? -(my_val) : (my_val)

//PID container objects
legCtrlStruct legCtrls[NUM_MOTOR_PIDS];
#ifdef PID_HARDWARE
//The vars for the DSP core PID controller have to decleared separately, due to the section attribute
pidHWvars legCtrlsHWvars[NUM_MOTOR_PIDS] __attribute__((section(".xbss, bss, xmemory")));
#endif


//Counter for blinking the red LED during motion
int blinkCtr;

//This is an option to force the PID outputs back to zero when there is no input.
//This was an attempt to stop bugs w/ motor twitching, or controller wandering.
//It may not be needed anymore.
#define PID_ZEROING_ENABLE 1

//Move queue variables, global
//TODO: move these into a move queue interface module
MoveQueue moveq;
moveCmdT currentMove, idleMove;
unsigned long currentMoveStart, moveExpire;

//BEMF related variables; we store a history of the last 3 values,
//but also provide variables for the "current" and "last" values for clarity
//in code below
//int bemf[NUM_MOTOR_PIDS]; //used to store the true, unfiltered speed
//int bemfLast[NUM_MOTOR_PIDS]; // Last post-median-filter value
//int bemfHist[NUM_MOTOR_PIDS][3]; //This is ONLY for applying the median filter to
int medianFilter3(int*);

//This is an array to map legCtrl controller to PWM output channels
//int legCtrlOutputChannels[NUM_MOTOR_PIDS];

//Global flag for wether or not the robot is in motion
volatile char inMotion;

//Function to be installed into T1, and setup function
static void SetupTimer1(void);
static void legCtrlServiceRoutine(void);  //To be installed with sysService
//The following local functions are called by the service routine:
static void serviceMoveQueue(void);
static void moveSynth();
static void serviceMotionPID();
static void updateBEMF();
static void setInitialOffset();

static int pwm_period;
static int max_pwm;

/////////        Leg Control ISR       ////////
/////////  Installed to Timer1 @ 1Khz  ////////
//void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
static void legCtrlServiceRoutine(void){
    serviceMoveQueue();
    moveSynth();         //TODO: port to synth module
    serviceMotionPID();  //Update controllers
}

static void SetupTimer1(void) {
    unsigned int T1CON1value, T1PERvalue;
    T1CON1value = T1_ON & T1_SOURCE_INT & T1_PS_1_1 & T1_GATE_OFF &
            T1_SYNC_EXT_OFF & T1_IDLE_CON;  //correct

    T1PERvalue = 0x9C40; //clock period = 0.001s = (T1PERvalue/FCY) (1KHz)
    //T1PERvalue = 0x9C40/2;
    //getT1_ticks() = 0;
    //OpenTimer1(T1CON1value, T1PERvalue);
    //ConfigIntTimer1(T1_INT_PRIOR_6 & T1_INT_ON);
    int retval;
    retval = sysServiceConfigT1(T1CON1value, T1PERvalue, T1_INT_PRIOR_6 & T1_INT_ON);
    //TODO: Put a soft trap here, conditional on retval
}


void legCtrlSetup() {
    int i;

    //Get maximum & saturation values
    //The factor of 2 is a quirk of the MicroChip PWM module, rising AND falling edges of PWM are counted
    pwm_period = 2*tiGetPWMPeriod();  //calculation of this value is left to the module that configures the motor control peripheral
    max_pwm = 2*tiGetPWMMax();

    //Setup for PID controllers
    for (i = 0; i < NUM_MOTOR_PIDS; i++) {
        //These pointers have to be assigned to the module-local variables here.
        //It has to be done this way, due to the section attribute on abcCoeffs and controlHists.
        legCtrls[i].controller.dspPID.abcCoefficients = legCtrlsHWvars[i].abcCoeffs; //Gains aren't SET here, only a pointer is assigned
        legCtrls[i].controller.dspPID.controlHistory = legCtrlsHWvars[i].controlHists;

        pidInitPIDObj(&(legCtrls[i].controller), LEG_DEFAULT_KP, LEG_DEFAULT_KI,
                LEG_DEFAULT_KD, LEG_DEFAULT_KAW, LEG_DEFAULT_KFF); //PID module init function
        
        //Set max and saturation values
        legCtrls[i].controller.satValPos = max_pwm;
        legCtrls[i].controller.satValNeg = -max_pwm;
        legCtrls[i].controller.maxVal = pwm_period;
        legCtrls[i].controller.minVal = -pwm_period;
    }

    //Set which PWM output each PID Object will correspond to
    legCtrls[0].outputChannel = OCTOROACH_LEG1_MOTOR_CHANNEL;
    legCtrls[1].outputChannel = OCTOROACH_LEG2_MOTOR_CHANNEL;

    SetupTimer1(); // Timer 1 @ 1 Khz
    int retval;
    retval = sysServiceInstallT1(legCtrlServiceRoutine);

    //Move Queue setup and initialization
    moveq = mqInit(32);
    idleMove = malloc(sizeof (moveCmdStruct));
    idleMove->inputL = 0;
    idleMove->inputR = 0;
    idleMove->duration = 0;
    idleMove->type = MOVE_SEG_IDLE;
    idleMove->params[0] = 0;
    idleMove->params[1] = 0;
    idleMove->params[2] = 0;
    idleMove->steeringType = STEERMODE_OFF;
    idleMove->steeringRate = 0;
    currentMove = idleMove;

    currentMoveStart = 0;
    moveExpire = 0;
    blinkCtr = 0;
    inMotion = 0;

    //This will set legCtrls[i].controller.inputOffset for i=0,1
    setInitialOffset();

    //Ensure controllers are reset to zero and turned off
    //External function used here since it will zero out the state
    pidSetInput(&(legCtrls[0].controller), 0);
    pidSetInput(&(legCtrls[1].controller), 0);
    legCtrls[0].controller.onoff = PID_OFF;
    legCtrls[1].controller.onoff = PID_OFF;

    //Set up filters and histories
    for (i = 0; i < NUM_MOTOR_PIDS; i++) {
        legCtrls[i].bemf = 0;
        legCtrls[i].bemfLast = 0;
        legCtrls[i].bemfHist[0] = 0;
        legCtrls[i].bemfHist[1] = 0;
        legCtrls[i].bemfHist[2] = 0;
    }
}

// Runs the PID controllers for the legs
void serviceMotionPID() {

    //Apply steering mixing, without overwriting anything
    int presteer[2] = {legCtrls[0].controller.input, legCtrls[1].controller.input};
    int poststeer[2] = {0, 0};
    steeringApplyCorrection(presteer, poststeer);
    legCtrls[0].controller.input = poststeer[0];
    legCtrls[1].controller.input = poststeer[1];

    updateBEMF();

    /////////// PID Section //////////
    int j;
    for (j = 0; j < NUM_MOTOR_PIDS; j++) {

        //pidobjs[0] : Left side
        //pidobjs[0] : Right side
        if (legCtrls[j].controller.onoff) {
            //TODO: Do we want to add provisions to track error, even when
            //the output is switched off?

#ifdef PID_SOFTWARE
            //Update values
            pidUpdate(&(motor_pidObjs[j]), bemf[j]);
#elif defined PID_HARDWARE
            //Apply scaling, update, remove scaling for consistency
            int temp;
            temp = legCtrls[j].controller.input; //Save unscaled input val
            legCtrls[j].controller.input *= MOTOR_PID_SCALER; //Scale input
            pidUpdate(&(legCtrls[j].controller), MOTOR_PID_SCALER* legCtrls[j].bemf);
            legCtrls[j].controller.input = temp;  //Reset unscaled input
#endif //PID_SOFTWWARE vs PID_HARDWARE

            //Set PWM duty cycle
            tiHSetDC(legCtrls[j].outputChannel, legCtrls[j].controller.output);

        }//end of if (on / off)
        else if (PID_ZEROING_ENABLE) { //if PID loop is off
            tiHSetDC(legCtrls[j].outputChannel, 0);
        }

    } // end of for(j)
}

void updateBEMF() {
    //Back EMF measurements are made automatically by coordination of the ADC, PWM, and DMA.
    //Copy to local variables. Not strictly neccesary, just for clarity.
    //This **REQUIRES** that the divider on the battery & BEMF circuits have the same ratio.
    legCtrls[0].bemf = adcGetVBatt() - adcGetBEMFL();
    legCtrls[1].bemf = adcGetVBatt() - adcGetBEMFR();

    //Apply median filter
    int i;
    for (i = 0; i < NUM_MOTOR_PIDS; i++) {

        legCtrls[i].bemfHist[2] = legCtrls[i].bemfHist[1]; //rotate first
        legCtrls[i].bemfHist[1] = legCtrls[i].bemfHist[0];
        legCtrls[i].bemfHist[0] = legCtrls[i].bemf; //include newest value
        legCtrls[i].bemf = medianFilter3(legCtrls[i].bemfHist); //Apply median filter
    }

    // IIR filter on BEMF: y[n] = 0.2 * y[n-1] + 0.8 * x[n]
    for (i = 0; i < NUM_MOTOR_PIDS; i++) {
        legCtrls[i].bemf = (2 * (long) legCtrls[i].bemfLast / 10) + 8 * (long) legCtrls[i].bemf / 10;
        legCtrls[i].bemfLast = legCtrls[i].bemf; //bemfLast will not be used after here, OK to set
    }

    //Subtract offset
    //This is relevant for IP2.5, since the motors can go in forward and reverse
    legCtrls[0].bemf -= legCtrls[0].controller.inputOffset;
    legCtrls[1].bemf -= legCtrls[1].controller.inputOffset;

    //Simple indicator if a leg is "in motion", via the yellow LED.
    //Not functionally necceasry; can be elimited to use the LED for something else.
    //if ((bemf[0] > 0) || (bemf[1] > 0)) {
    //    LED_YELLOW = 1;
    //} else {
    //    LED_YELLOW = 0;
    //}
}

void serviceMoveQueue(void) {

    //Blink red LED when executing move program
    if (currentMove != idleMove) {
        inMotion = 1;
        if (blinkCtr == 0) {
            blinkCtr = 100;
            LED_RED = ~LED_RED;
        }
        blinkCtr--;
    }

    //Service Move Queue if not empty
    if (!mqIsEmpty(moveq)) {
        inMotion = 1;
        if ((currentMove == idleMove) || (getT1_ticks() >= moveExpire)) {
            currentMove = mqPop(moveq);
            //MOVE_SEG_LOOP_DECL only needs to appear once
            //This will set up queue looping
            if (currentMove->type == MOVE_SEG_LOOP_DECL) {
                mqLoopingOnOff(1);
                currentMove = mqPop(moveq);
            }
            //Stop queue looping
            if (currentMove->type == MOVE_SEG_LOOP_CLEAR) {
                mqLoopingOnOff(0);
                currentMove = mqPop(moveq);
            }
            //Remove all remaining items from the queue
            if (currentMove->type == MOVE_SEG_QFLUSH) {
                while (mqPop(moveq)); //Terminate on NULL return, flushing queue
                currentMove = idleMove;
            }
            //TODO: handle NULL return from mqPop
            moveExpire = getT1_ticks() + currentMove->duration;
            currentMoveStart = getT1_ticks();

            ///// Steering settings from Move Queue
            steeringSetInput(currentMove->steeringRate); //THIS TURNS THE STEERING ON!
            if (currentMove->steeringType == STEERMODE_OFF) {
                steeringOff();
            }
            steeringSetMode(currentMove->steeringType);
            //This is not strictly correct when using STEER_MODE_YAW,
            //Since we are labeling it as a "rate"
            steeringSetInput(currentMove->steeringRate);

            //If we are no on an Idle move, turn on controllers
            if (currentMove->type != MOVE_SEG_IDLE) {
                legCtrls[0].controller.onoff = PID_ON;
                legCtrls[1].controller.onoff = PID_ON;
            }
        }
    }//Move Queue is empty
        //Else if: exipry of last move in queue, return to idle
    else if ((getT1_ticks() >= moveExpire) && currentMove != idleMove) {
        //No more moves, go back to idle
        currentMove = idleMove;
        pidSetInput(&(legCtrls[0].controller), 0);
        legCtrls[0].controller.onoff = PID_OFF;
        pidSetInput(&(legCtrls[1].controller), 0);
        legCtrls[1].controller.onoff = PID_OFF;
        moveExpire = 0;
        inMotion = 0; //for sleep, synthesis
        steeringOff();
    }
}

static void moveSynth() {
    //Move segment synthesis
    long ySL = currentMove->inputL; //store in local variable to limit lookups
    long ySR = currentMove->inputR; // "
    int yL = 0;
    int yR = 0;

    if (inMotion) {
        if (currentMove->type == MOVE_SEG_IDLE) {
            yL = 0;
            yR = 0;
        }
        if (currentMove->type == MOVE_SEG_CONSTANT) {
            yL = ySL;
            yR = ySR;

        }

        if (currentMove->type == MOVE_SEG_RAMP) {
            long rateL = (long) currentMove->params[0];
            long rateR = (long) currentMove->params[1];
            //Do division last to prevent integer math underflow
            yL = rateL * ((long) getT1_ticks() - (long) currentMoveStart) / 1000 + ySL;
            yR = rateR * ((long) getT1_ticks() - (long) currentMoveStart) / 1000 + ySR;
        }
        if (currentMove->type == MOVE_SEG_SIN) {
            //float temp = 1.0/1000.0;
            float amp = (float) currentMove->params[0];
            //float F = (float)currentMove->params[1] / 1000;
            float F = (float) currentMove->params[1] * 0.001;
#define BAMS16_TO_FLOAT 1/10430.367658761737
            float phase = BAMS16_TO_FLOAT * (float) currentMove->params[2]; //binary angle
            //Must be very careful about underflow & overflow here!
            float fyL = amp * sin(2 * 3.1415 * F * (float) (getT1_ticks() - currentMoveStart)*0.001 - phase) + ySL;
            float fyR = amp * sin(2 * 3.1415 * F * (float) (getT1_ticks() - currentMoveStart)*0.001 - phase) + ySR;


            //fractional arg = 2*(long)F*((long)getT1_ticks()-(long)currentMoveStart) *
            //fractional temp = _Q15sinPI(arg);
            //fractional wave = (int)((long)temp*(long)amp >> 15);

            //Clipping
            int temp = (int) fyL;
            if (temp < 0) {
                temp = 0;
            }
            yL = (unsigned int) temp;
            temp = (int) fyR;
            if (temp < 0) {
                temp = 0;
            }
            yR = (unsigned int) temp;
            //unsigned int yL = amp*sin(arg) + ySL;
        }

        //Transfer calculated setpoints to the controllers
        legCtrls[0].controller.input = yL;
       legCtrls[1].controller.input = yR;

    }
    //Note here that pidObjs[n].input is not set if !inMotion, in case another behavior wants to
    // set it.
}



//Poor implementation of a median filter for a 3-array of values

int medianFilter3(int* a) {
    int b[3] = {a[0], a[1], a[2]};
    int temp;

    //Implemented through 3 compare-exchange operations, increasing index
    if (b[0] > b[1]) {
        temp = b[1];
        b[1] = b[0];
        b[0] = temp;
    }
    if (a[0] > a[2]) {
        temp = b[2];
        b[2] = b[0];
        b[0] = temp;
    }
    if (a[1] > a[2]) {
        temp = b[2];
        b[2] = b[1];
        b[1] = temp;
    }

    return b[1];
}

void legCtrlSetInput(unsigned int num, int val) {
    pidSetInput(&(legCtrls[num].controller), val);
}

void legCtrlOnOff(unsigned int num, unsigned char state) {
   legCtrls[num].controller.onoff = state;
}

void legCtrlSetGains(unsigned int num, int Kp, int Ki, int Kd, int Kaw, int ff) {
    pidSetGains(&(legCtrls[num].controller), Kp, Ki, Kd, Kaw, ff);
}

static void setInitialOffset() {
    int i;

    //Offsets are expected to be ~511 counts for motor stationary.
    long offset1 = 0;
    long offset2 = 0;
    for (i = 0; i < 8; i++) {
        offset1 += adcGetBEMFL();
        offset2 += adcGetBEMFR();
        delay_ms(2);
    }

    offset1 = offset1 >> 3; // fast div by 8
    legCtrls[0].controller.inputOffset = offset1;
    offset1 = offset2 >> 3;
    legCtrls[1].controller.inputOffset = offset2;
}