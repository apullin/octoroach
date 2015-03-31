/*
 * Name: UpdatePID.c
 * Desc: Control code to compute the new input to the plant
 * Date: 2009-04-03
 * Author: AMH
   modified to include hall effect sensor by RSF.
 * modified Dec. 2011 to include telemetry capture
 * modified Jan. 2012 to include median filter on back emf
 * modified Jan. 2013 to include AMS Hall encoder, and MPU 6000 gyro
 */

#include <xc.h>
//dsPIC library
#include "timer.h"
#include "pwm.h"
#include "adc.h"
#include <stdlib.h> // for malloc
//imageproc-lib and library includes
#include "pid-ip2.5.h"
#include "dfmem.h"
#include "adc_pid.h"
#include "sclock.h"
#include "ams-enc.h"
#include "tih.h"
#include "mpu6000.h"
#include "uart_driver.h"
#include "ppool.h"
#include "dfmem.h"
#include "telem.h"
#include "sys_service.h"


#define MC_CHANNEL_PWM1     1
#define MC_CHANNEL_PWM2     2
#define MC_CHANNEL_PWM3     3
#define MC_CHANNEL_PWM4     4

//#define HALFTHROT 10000
#define HALFTHROT 2000
#define FULLTHROT 2*HALFTHROT
// MAXTHROT has to allow enough time at end of PWM for back emf measurement
// was 3976
#define MAXTHROT 3800

#define ABS(my_val) ((my_val) < 0) ? -(my_val) : (my_val)

// PID control structure
pidPos pidObjs[NUM_PIDS];

// structure for reference velocity for leg
pidVelLUT pidVel[NUM_PIDS*NUM_BUFF];
pidVelLUT* activePID[NUM_PIDS]; //Pointer arrays for stride buffering
pidVelLUT* nextPID[NUM_PIDS];

//#define T1_MAX 0xffffff  // max before rollover of 1 ms counter
// may be glitch in longer missions at rollover
//volatile unsigned long t1_ticks;
unsigned long lastMoveTime;

// 2 last readings for median filter
int measLast1[NUM_PIDS];
int measLast2[NUM_PIDS];
int bemf[NUM_PIDS];

int medianFilter3(int*);

//Service routine function
static void pidip25ServiceRoutine();

//Private functions
static void setInitialOffset(unsigned int samples);

/////////        Leg Control ISR       ////////
/////////  Installed to Timer1 @ 1Khz  ////////

static void SetupTimer1(void) {
    unsigned int T1CON1value, T1PERvalue;
    T1CON1value = T1_ON & T1_SOURCE_INT & T1_PS_1_1 & T1_GATE_OFF &
            T1_SYNC_EXT_OFF & T1_IDLE_CON;  //correct

    T1PERvalue = 0x9C40; //clock period = 0.001s = (T1PERvalue/FCY) (1KHz)
    int retval;
    retval = sysServiceConfigT1(T1CON1value, T1PERvalue, T1_INT_PRIOR_5 & T1_INT_ON);
    //TODO: Put a soft trap here, conditional on retval
}


// -------------------------------------------
// called from main()

void pidSetup() {
    int i;
    for (i = 0; i < NUM_PIDS; i++) {
        initPIDObjPos(&(pidObjs[i]), DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DEFAULT_KAW, DEFAULT_FF);
    }
    initPIDVelProfile();
    SetupTimer1(); // main interrupt used for leg motor PID

    lastMoveTime = 0;

    //TODO: This should be generalized fo there is no sense of "left" and "right" here
    pidObjs[LEFT_LEGS_PID_NUM].output_channel  = LEFT_LEGS_TIH_CHAN;
    pidObjs[LEFT_LEGS_PID_NUM].p_state_flip    = LEFT_LEGS_ENC_FLIP;
    pidObjs[LEFT_LEGS_PID_NUM].encoder_num     = LEFT_LEGS_ENC_NUM;
    pidObjs[LEFT_LEGS_PID_NUM].pwm_flip        = LEFT_LEGS_PWM_FLIP;

    pidObjs[RIGHT_LEGS_PID_NUM].output_channel = RIGHT_LEGS_TIH_CHAN;
    pidObjs[RIGHT_LEGS_PID_NUM].p_state_flip   = RIGHT_LEGS_FLIP;
    pidObjs[RIGHT_LEGS_PID_NUM].encoder_num    = RIGHT_LEGS_ENC_NUM;
    pidObjs[RIGHT_LEGS_PID_NUM].pwm_flip       = RIGHT_LEGS_PWM_FLIP;

    // Initialize PID structures before starting Timer1
    pidSetInput(LEFT_LEGS_PID_NUM, 0);
    pidSetInput(RIGHT_LEGS_PID_NUM, 0);

    setInitialOffset(16); //2ms delay between samples, so 32ms calib time

    //EnableIntT1; // turn on pid interrupts
    SetupTimer1(); // Timer 1 @ 1 Khz
    int retval;
    retval = sysServiceInstallT1(pidip25ServiceRoutine);
}


//Returns pointer to non-active buffer

pidVelLUT* otherBuff(pidVelLUT* array, pidVelLUT* ptr) {
    if (ptr >= &(array[NUM_PIDS])) {
        return ptr - NUM_PIDS;
    } else {
        return ptr + NUM_PIDS;
    }
}

// ----------   all the initializations  -------------------------
// set expire time for first segment in pidSetInput - use start time from MoveClosedLoop
// set points and velocities for one revolution of leg
// called from pidSetup()

void initPIDVelProfile() {
    int i, j;
    pidVelLUT* tempPID;
    for (j = 0; j < NUM_PIDS; j++) {
        pidObjs[j].index = 0; // point to first velocity
        pidObjs[j].interpolate = 0;
        pidObjs[j].leg_stride = 0; // set initial leg count
        activePID[j] = &(pidVel[j]); //Initialize buffer pointers
        nextPID[j] = NULL;
        tempPID = otherBuff(pidVel, activePID[j]);
        for (i = 0; i < NUM_VELS; i++) {
            tempPID->interval[i] = 100;
            tempPID->delta[i] = 0;
            tempPID->vel[i] = 0;
        }
        tempPID->onceFlag = 0;
        nextPID[j] = tempPID;
        pidObjs[j].p_input = 0; // initialize first set point 
        pidObjs[j].v_input = (int) (((long) pidVel[j].vel[0] * K_EMF) >> 8); //initialize first velocity, scaled
    }
}


// called from cmd.c

void setPIDVelProfile(int pid_num, int *interval, int *delta, int *vel, int onceFlag) {
    pidVelLUT* tempPID;
    int i;
    tempPID = otherBuff(pidVel, activePID[pid_num]);
    for (i = 0; i < NUM_VELS; i++) {
        tempPID->interval[i] = interval[i];
        tempPID->delta[i] = delta[i];
        tempPID->vel[i] = vel[i];
    }
    tempPID->onceFlag = onceFlag;
    if (activePID[pid_num]->onceFlag == 0) {
        nextPID[pid_num] = tempPID;
    }
}


// called from pidSetup()

void initPIDObjPos(pidPos *pid, int Kp, int Ki, int Kd, int Kaw, int ff) {
    pid->p_input = 0;
    pid->v_input = 0;
    pid->p = 0;
    pid->i = 0;
    pid->d = 0;
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Kaw = Kaw;
    pid->feedforward = 0;
    pid->output = 0;
    pid->onoff = 0;
    pid->p_error = 0;
    pid->v_error = 0;
    pid->i_error = 0;

    pid->p_state_flip = 0; //default to no flip
    pid->output_channel = 0;
    pid->inputOffset = 0;

    pid->bemfHist[0] = 0; pid->bemfHist[1] = 0; pid->bemfHist[2] = 0;
}



// called from set thrust closed loop, etc. Thrust

void pidSetInput(int pid_num, int input_val) {
    unsigned long temp;
    /*      ******   use velocity setpoint + throttle for compatibility between Hall and Pullin code *****/
    /* otherwise, miss first velocity set point */
    pidObjs[pid_num].v_input = input_val + (int) (((long) pidVel[pid_num].vel[0] * K_EMF) >> 8); //initialize first velocity ;
    pidObjs[pid_num].start_time = getT1_ticks();
    //zero out running PID values
    pidObjs[pid_num].i_error = 0;
    pidObjs[pid_num].p = 0;
    pidObjs[pid_num].i = 0;
    pidObjs[pid_num].d = 0;
    //Seed the median filter
    measLast1[pid_num] = input_val;
    measLast2[pid_num] = input_val;

    // set initial time for next move set point
    /*   need to set index =0 initial values */
    /* position setpoints start at 0 (index=0), then interpolate until setpoint 1 (index =1), etc */
    temp = 0;
    pidObjs[pid_num].expire = temp + (long) pidVel[pid_num].interval[0]; // end of first interval
    pidObjs[pid_num].interpolate = 0;
    /*	pidObjs[pid_num].p_input += pidVel[pid_num].delta[0];	//update to first set point
     ***  this should be set only after first .expire time to avoid initial transients */
    pidObjs[pid_num].index = 0; // reset setpoint index
    // set first move at t = 0
    //	pidVel[0].expire = temp;   // right side
    //	pidVel[1].expire = temp;   // left side

}

void pidStartTimedTrial(unsigned int run_time) {
    unsigned long temp;

    temp = getT1_ticks(); // need atomic read due to interrupt
    pidObjs[0].run_time = run_time;
    pidObjs[1].run_time = run_time;
    pidObjs[0].start_time = temp;
    pidObjs[1].start_time = temp;
    if ((temp + (unsigned long) run_time) > lastMoveTime) {
        lastMoveTime = temp + (unsigned long) run_time;
    } // set run time to max requested time
}

// from cmd.c  PID set gains

void pidSetGains(int pid_num, int Kp, int Ki, int Kd, int Kaw, int ff) {
    pidObjs[pid_num].Kp = Kp;
    pidObjs[pid_num].Ki = Ki;
    pidObjs[pid_num].Kd = Kd;
    pidObjs[pid_num].Kaw = Kaw;
    pidObjs[pid_num].feedforward = ff;
}

void pidOn(int pid_num) {
    pidObjs[pid_num].onoff = PID_ON;
    //t1_ticks = 0;
    //sysService does not support timer zeroing
}

void pidOff(int pid_num) {
    pidObjs[pid_num].onoff = PID_OFF;
    //t1_ticks = 0;
    //sysService does not support timer zeroing
}

// zero position setpoint for both motors (avoids big offset errors)

void pidZeroPos(int pid_num) {
    // disable interrupts to reset state variables
    DisableIntT1; // turn off pid interrupts
    amsEncoderResetPos(); //  reinitialize rev count and relative zero encoder position for both motors
    pidObjs[pid_num].p_state = 0;
    // reset position setpoint as well
    pidObjs[pid_num].p_input = 0;
    pidObjs[pid_num].v_input = 0;
    pidObjs[pid_num].leg_stride = 0; // strides also reset
    EnableIntT1; // turn on pid interrupts
}


/*****************************************************************************************/
/*****************************************************************************************/
/*********************** Stop Motor and Interrupts *********************************************/
/*****************************************************************************************/

/*****************************************************************************************/
void EmergencyStop(void) {
    pidSetInput(0, 0);
    pidSetInput(1, 0);
    DisableIntT1; // turn off pid interrupts
    SetDCMCPWM(MC_CHANNEL_PWM1, 0, 0); // set PWM to zero
    SetDCMCPWM(MC_CHANNEL_PWM2, 0, 0);
}


// -------------------------   control  loop section  -------------------------------

/*********************** Motor Control Interrupt *********************************************/
/*****************************************************************************************/
/*****************************************************************************************/

/* update setpoint  only leg which has run_time + start_time > t1_ticks */
/* turn off when all PIDs have finished */
//static volatile unsigned char interrupt_count = 0;
//static volatile unsigned char telemetry_count = 0;
extern volatile MacPacket uart_tx_packet;
extern volatile unsigned char uart_tx_flag; //TODO: separate this UART kruft


static void pidip25ServiceRoutine() {

    unsigned long t1_ticks = getT1_ticks();

    int j = 0;

    pidGetState(); // always update state, even if motor is coasting
    for (j = 0; j < NUM_PIDS; j++) {
        // only update tracking setpoint if time has not yet expired
        if (pidObjs[j].onoff) {
            if (pidObjs[j].timeFlag) {
                if (pidObjs[j].start_time + pidObjs[j].run_time >= t1_ticks) {
                    pidGetSetpoint(j);
                }
                if (t1_ticks > lastMoveTime) { // turn off if done running all legs
                    pidObjs[0].onoff = 0;
                    pidObjs[1].onoff = 0;
                }
            } else {
                pidGetSetpoint(j);
            }
        }
    }
    if (pidObjs[0].mode == PID_MODE_CONTROLED) {
        pidSetControl();
    } else if (pidObjs[0].mode == PID_MODE_PWMPASS) {
        tiHSetDC(pidObjs[0].output_channel, pidObjs[0].pwmDes);
        tiHSetDC(pidObjs[1].output_channel, pidObjs[1].pwmDes);
    }

    LED_3 = 0;


    //Previous method: 5khz timer with 5 time slices
    //Instead, functions wil just be called in a reasonable order here.
    //This is NOT good form; each one of these calls should be in their own
    //module, and install with sysServiceT1


    //This used to be done in 1 of 5 time slices of a 5khz interrupt
    amsEncoderStartAsyncRead();

    //This used to be done in 1 of 5 time slices of a 5khz interrupt
    //This should be moved to an IMU module
    mpuBeginUpdate();

    //This used to be done in 1 of 5 time slices of a 5khz interrupt
    //This should be moved to a telem module.
    telemSaveNow();

}

// update desired velocity and position tracking setpoints for each leg

void pidGetSetpoint(int j) {
    int index;
    index = pidObjs[j].index;
    // update desired position between setpoints, scaled by 256
    pidObjs[j].interpolate += (long) activePID[j]->vel[index];

    if (getT1_ticks() >= pidObjs[j].expire) { // time to reach previous setpoint has passed
        pidObjs[j].interpolate = 0;
        pidObjs[j].p_input += activePID[j]->delta[index]; //update to next set point
        pidObjs[j].index++;

        if (pidObjs[j].index >= NUM_VELS) {
            pidObjs[j].index = 0;
            pidObjs[j].leg_stride++; // one full leg revolution
            /**** maybe need to handle round off in position set point ***/
            checkSwapBuff(j);
        }
        pidObjs[j].expire += activePID[j]->interval[pidObjs[j].index]; // expire time for next interval
        pidObjs[j].v_input = (activePID[j]->vel[pidObjs[j].index]); //update to next velocity
    }
}

void checkSwapBuff(int j) {
    if (nextPID[j] != NULL) { //Swap pointer if not null
        if (nextPID[j]->onceFlag == 1) {
            pidVelLUT* tempPID;
            CRITICAL_SECTION_START;
            tempPID = activePID[j];
            activePID[j] = nextPID[j];
            nextPID[j] = tempPID;
            CRITICAL_SECTION_END;
        } else {
            CRITICAL_SECTION_START;
            activePID[j] = nextPID[j];
            nextPID[j] = NULL;
            CRITICAL_SECTION_END;
        }
    }
}


/* update state variables including motor position and velocity */

void pidGetState() {
    int i;
    long p_state;
    int enc_num;
    int encPosition, encOticks;
    unsigned int encOffset;

    unsigned long time_start, time_end;

    // choose velocity estimate
#ifndef VEL_BEMF    // use first difference on position for velocity estimate
    long oldpos[NUM_PIDS], velocity;
    for (i = 0; i < NUM_PIDS; i++) {
        oldpos[i] = pidObjs[i].p_state;
    }
#endif


    //TODO: Change BEMF getter functions to function pointers, make settable
    time_start = sclockGetTime();
    bemf[0] = pidObjs[0].inputOffset - adcGetMotorA(); // watch sign for A/D? unsigned int -> signed?
    bemf[1] = pidObjs[1].inputOffset - adcGetMotorB(); // MotorB

    // only works to +-32K revs- might reset after certain number of steps? Should wrap around properly
    for (i = 0; i < NUM_PIDS; i++) {

        enc_num = pidObjs[i].encoder_num;
        
        encPosition = amsEncoderGetPos(enc_num);
        encOticks = amsEncoderGetOticks(enc_num);
        encOffset = amsEncoderGetOffset(enc_num);

        p_state =  (long)encPosition << 2; // pos 14 bits 0x0 -> 0x3fff
        p_state = p_state - ((long)encOffset << 2); // subtract offset to get zero position
        p_state = p_state + ((long)encOticks << 16);

        pidObjs[i].p_state = p_state;
        
        if(pidObjs[i].p_state_flip){
            pidObjs[i].p_state = -pidObjs[i].p_state;
        }

    }

    time_end = sclockGetTime() - time_start;


#ifndef VEL_BEMF    // use first difference on position for velocity estimate
    for (i = 0; i < NUM_PIDS; i++) {
        velocity = pidObjs[i].p_state - oldpos[i]; // Encoder ticks per ms
        if (velocity > 0x7fff) velocity = 0x7fff; // saturate to int
        if (velocity < -0x7fff) velocity = -0x7fff;
        pidObjs[i].v_state = (int) velocity;
    }
#endif

    // choose velocity estimate

#ifdef VEL_BEMF

    //Rotate BEMF history
    pidObjs[0]->bemfHist[2] = pidObjs[0]->bemfHist[1];
    pidObjs[0]->bemfHist[1] = pidObjs[0]->bemfHist[0];

    pidObjs[1]->bemfHist[2] = pidObjs[0]->bemfHist[1];
    pidObjs[1]->bemfHist[1] = pidObjs[0]->bemfHist[0];

    pidObjs[0]->bemfHist[0] = bemf[0];
    pidObjs[1]->bemfHist[0] = bemf[1];

    //Get motor speed reading on every interrupt - A/D conversion triggered by PWM timer to read Vm when transistor is off
    // when motor is loaded, sometimes see motor short so that  bemf=offset voltage
    // get zero sometimes - open circuit brush? Hence try median filter
    

    pidObjs[0].v_state = medianFilter3(pidObjs[1]->bemfHist);
    pidObjs[1].v_state = medianFilter3(pidObjs[1]->bemfHist);

#endif
}

void pidSetControl() {
    int j;
    // 0 = right side
    for (j = 0; j < NUM_PIDS; j++) { //pidobjs[0] : right side
        // p_input has scaled velocity interpolation to make smoother
        // p_state is [16].[16]
        pidObjs[j].p_error = pidObjs[j].p_input + pidObjs[j].interpolate - pidObjs[j].p_state;
        pidObjs[j].v_error = pidObjs[j].v_input - pidObjs[j].v_state; // v_input should be revs/sec
        //Update values
        UpdatePID(&(pidObjs[j]));
    } // end of for(j)

    if (pidObjs[0].onoff && pidObjs[1].onoff) // both motors on to run
    {
        if(pidObjs[0].pwm_flip){
            tiHSetDC(pidObjs[0].output_channel, -pidObjs[0].output);
        }
        else{
            tiHSetDC(pidObjs[0].output_channel, pidObjs[0].output);
        }
        
        if(pidObjs[1].pwm_flip){
            tiHSetDC(pidObjs[1].output_channel, -pidObjs[1].output);
        }
        else{
            tiHSetDC(pidObjs[1].output_channel, pidObjs[1].output);
        }
    }
    else // turn off motors if PID loop is off
    {
        tiHSetDC(pidObjs[0].output_channel, 0);
        tiHSetDC(pidObjs[1].output_channel, 0);
    }
}

void UpdatePID(pidPos *pid) {
    pid->p = ((long) pid->Kp * pid->p_error) >> 12; // scale so doesn't over flow
    pid->i = (long) pid->Ki * pid->i_error >> 12;
    pid->d = (long) pid->Kd * (long) pid->v_error;
    // better check scale factors

    pid->preSat = pid->feedforward + pid->p +
            ((pid->i) >> 4) + // divide by 16
            (pid->d >> 4); // divide by 16
    pid->output = pid->preSat;

    /* i_error say up to 1 rev error 0x10000, X 256 ms would be 0x1 00 00 00
        scale p_error by 16, so get 12 bit angle value*/
    pid-> i_error = (long) pid-> i_error + ((long) pid->p_error >> 4); // integrate error
    // saturate output - assume only worry about >0 for now
    // apply anti-windup to integrator
    if (pid->preSat > MAXTHROT) {
        pid->output = MAXTHROT;
        pid->i_error = (long) pid->i_error +
                (long) (pid->Kaw) * ((long) (MAXTHROT) - (long) (pid->preSat))
                / ((long) GAIN_SCALER);
    }
    if (pid->preSat < -MAXTHROT) {
        pid->output = -MAXTHROT;
        pid->i_error = (long) pid->i_error +
                (long) (pid->Kaw) * ((long) (MAXTHROT) - (long) (pid->preSat))
                / ((long) GAIN_SCALER);
    }
}

//TODO: Controller design, this function was created specifically to remove existing externs.
long pidGetPState(unsigned int channel) {
    if (channel < NUM_PIDS) {
        return pidObjs[channel].p_state;
    } else {
        return 0;
    }
}

//TODO: Controller design, this function was created specifically to remove existing externs.
void pidSetPInput(unsigned int channel, long p_input) {
    if (channel < NUM_PIDS) {
        pidObjs[channel].p_input = p_input;
    }
}

//TODO: Controller design, this function was created specifically to remove existing externs.
void pidStartMotor(unsigned int channel){
    if (channel < NUM_PIDS) {
        pidObjs[channel].timeFlag = 0;
        pidSetInput(channel, 0);
        pidObjs[channel].p_input = pidObjs[channel].p_state;
        pidOn(channel);
    }

}

//TODO: Controller design, this function was created specifically to remove existing externs.
void pidSetTimeFlag(unsigned int channel, char val){
    if (channel < NUM_PIDS) {
        pidObjs[channel].timeFlag = val;
    }
}

//TODO: Controller design, this function was created specifically to remove existing externs.
void pidSetMode(unsigned int channel, char mode){
    if (channel < NUM_PIDS) {
        pidObjs[channel].mode = mode;
    }
}

void pidSetPWMDes(unsigned int channel, int pwm){
    if (channel < NUM_PIDS) {
        pidObjs[channel].pwmDes = pwm;
    }
}


static void setInitialOffset(unsigned int samples) {
    //For IP2.5, it is expected that the offsets for idling motors should be about 511 ADC counts
    // See wiki page on circuit for more details

    int i;

    //Offsets are expected to be ~511 counts for motor stationary.
    long offsets[NUM_PIDS];

    delay_ms(10); //Settling time.

    //Accumulate 8 readings to average out
    for (i = 0; i < samples; i++) {
        offsets[0] += adcGetMotorA();
        offsets[1] += adcGetMotorB();
        delay_ms(2);
    }

    for(i = 0; i<NUM_PIDS; i++){
        offsets[i] = offsets[i] >> 3; // fast div by 8
        pidObjs[i].inputOffset = offsets[i]; //store
    }

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