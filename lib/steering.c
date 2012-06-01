#include "pid.h"
#include "timer.h"
#include "gyro.h"
#include "steering.h"
#include "stopwatch.h"
#include "dfmem.h"
#include "telem.h"
#include "move_queue.h"
#include "xl.h"
//#include "orient.h"
#include "dfilter_avg.h"
#include "pid_hw.h"

#define ABS(my_val) ((my_val) < 0) ? -(my_val) : (my_val)

long gyro_accum;
int j;
pidT steeringPID;
int steeringIsOn;

tPID steering_hwPID;
fractional steering_abcCoeffs[3] __attribute__((section(".xbss, bss, xmemory")));
fractional steering_controlHists[3] __attribute__((section(".ybss, bss, ymemory")));

//Averaging filter structures for gyroscope data
//Initialzied in setup.
//filterAvgInt_t gyroXavg, gyroYavg, gyroZavg;
filterAvgInt_t gyroZavg;
#define GYRO_AVG_SAMPLES 	32

#define GYRO_DRIFT_THRESH 3

//unsigned int orientSkip = 0;

unsigned int steeringMode;

//Gyro offsets
//extern int offsx, offsy, offsz;

//extern pidT pidObjs[NUM_PIDS];
//extern int bemf[NUM_PIDS];

extern moveCmdT currentMove, idleMove;

//This should be replaced by a proper system clock, and a getter functions
extern unsigned long t1_ticks; //needed to calculate new runtimes

void steeringSetup(void) {

    gyro_accum = 0;
    j = 0;

#ifdef PID_HARDWARE
    //Create PID controller object
    pidCreate(&steering_hwPID, steering_abcCoeffs, steering_controlHists);
    initPIDObj(&steeringPID, STEERING_KP, STEERING_KI, STEERING_KD, STEERING_KAW, 0);
#else
    initPIDObj(&steeringPID, STEERING_KP, STEERING_KI, STEERING_KD, STEERING_KAW, 0);
#endif
    setSteeringAngRate(0);

    ///// Timer 5 setup, Steering ISR, 300Hz /////
    // period value = Fcy/(prescale*Ftimer)
    unsigned int con_reg, period;
    // prescale 1:64
    con_reg = T5_ON & T5_IDLE_CON & T5_GATE_OFF & T5_PS_1_64 & T5_SOURCE_INT;
    // Period is set so that period = 5ms (200Hz), MIPS = 40
    //period = 3125; // 200Hz
    period = 2083; // ~300Hz
    OpenTimer5(con_reg, period);
    ConfigIntTimer5(T5_INT_PRIOR_5 & T5_INT_ON);
    //////////////////////////////////////////////


    //Averaging filter setup:
    //filterAvgCreate(&gyroXavg, GYRO_AVG_SAMPLES);
    //filterAvgCreate(&gyroYavg, GYRO_AVG_SAMPLES);
    filterAvgCreate(&gyroZavg, GYRO_AVG_SAMPLES);

    steeringIsOn = 1;
    steeringMode = STEERMODE_DECREASE;
}

void __attribute__((interrupt, no_auto_psv)) _T5Interrupt(void) {

    // Steering update ISR handler
    steeringHandleISR();

    // Section for saving telemetry data to flash
    // Uses telemSkip as a divisor to T5.
    telemISRHandler();

    _T5IF = 0;
}

void setSteeringAngRate(int angRate) {
    _T5IE = 0;
    steeringPID.input = angRate;
    _T5IE = 1;
}

//This should be updated to use a generic PID module, DSP PID.

void UpdatePIDSteering(pidT *pid, int y) {
    pid->p = (long) pid->Kp * pid->error;
    pid->i = (long) pid->Ki * pid->iState;
    //Filtered derivative action applied directly to measurement
    pid->d = ((long) pid->Kd * (long) pid->d * (long) STEERING_GAIN_SCALER) / ((long) pid->Kd + (long) pid->Kp * (long) pid->N) -
            ((long) pid->Kd * (long) pid->Kp * (long) pid->N * ((long) y - (long) pid->y_old)) /
            ((long) pid->Kd + (long) pid->Kp * (long) pid->N);
    pid->d = 0;

    pid->preSat = (pid->p + pid->i + pid->d) / (long) STEERING_GAIN_SCALER;

    if (pid->preSat > STEERING_SAT) {
        pid->output = STEERING_SAT;
    } else {
        pid->output = pid->preSat;
    }

    pid->iState += (long) (pid->error) + ((long) (pid->Kaw) *
            ((long) (pid->output) - (long) (pid->preSat))) / (long) STEERING_GAIN_SCALER;
    pid->y_old = y;
}

void steeringSetGains(int Kp, int Ki, int Kd, int Kaw, int ff) {
    steeringPID.Kp = Kp;
    steeringPID.Ki = Ki;
    steeringPID.Kd = Kd;
    steeringPID.Kaw = Kaw;
    steeringPID.feedforward = ff;
    //If we are using the DSP core PID, we need to recalculate gain coeffs
#ifdef PID_HARDWARE
    //Gains are retrieved from the PID container object,
    //and need special setup for the DSP type PID calculation
    pidSetFracCoeffs(&steering_hwPID, steeringPID.Kp, steeringPID.Ki,
    steeringPID.Kd);
#endif
}

void steeringSetMode(unsigned int sm) {
    steeringMode = sm;
}

void steeringHandleISR() {

    //int gyroAvg[3];
    int gyroAvgZ;
    int gyroData[3];
    int gyroOffsets[3];

    gyroGetXYZ((unsigned char*) gyroData);
    gyroGetOffsets(gyroOffsets);

    //filterAvgUpdate(&gyroXavg,gyroData[0] - gyroOffsets[0]);
    //filterAvgUpdate(&gyroYavg,gyroData[1] - gyroOffsets[1]);
    filterAvgUpdate(&gyroZavg, gyroData[2] - gyroOffsets[2]);

    //gyroAvg[0] = filterAvgCalc(&gyroXavg);
    //gyroAvg[1] = filterAvgCalc(&gyroYavg);
    gyroAvgZ = filterAvgCalc(&gyroZavg);

    //Threshold filter on gyro to account for minor drift
    /*int i;
    for(i=0; i< 3; i++){
            if(gyroAvg[i] < 0){
                    if(-gyroAvg[i] < GYRO_DRIFT_THRESH){ gyroAvg[i] = 0;}
            }
            else{
                    if(gyroAvg[i] < GYRO_DRIFT_THRESH){ gyroAvg[i] = 0;}
            }
    }*/
    if (ABS(gyroAvgZ < GYRO_DRIFT_THRESH)) {
        gyroAvgZ = 0;
    }

    //Update the setpoints
    //if((currentMove->inputL != 0) && (currentMove->inputR != 0)){
    if (currentMove != idleMove) {
        //Only update steering controller if we are in motion
#ifdef PID_SOFTWARE
        steeringPID.error = steeringPID.input - gyroAvg[2];
        UpdatePIDSteering(&steeringPID, gyroAvg[2]);
#elif defined PID_HARDWARE
        int temp = 0;
        steering_hwPID.controlReference =
                STEERING_PID_ERR_SCALER * steeringPID.input;
        temp = pidRun(&steering_hwPID, STEERING_PID_ERR_SCALER * gyroAvgZ);
        //Saturation
        if ((int) (steering_hwPID.controlOutput) < 0) {
            temp = 0;
        }
        if ((int) (steering_hwPID.controlOutput) > STEERING_SAT) {
            temp = STEERING_SAT;
        }
        //Manually update the output tracking variable in the container,
        //since we are using the DSP controller for calculation
        steeringPID.output = temp;
#endif   //PID_SOFTWWARE vs PID_HARDWARE
    }
    //Now the output correction is stored in steeringPID.output,
    //which will be read later when the steering mixing is done.
}

void steeringApplyCorrection(int* inputs, int* outputs) {
    int left, right;
    //Steering Mixing
    //left = currentMove->inputL;
    //right = currentMove->inputR;
    left = inputs[0];
    right = inputs[1];

    if (steeringPID.onoff == 1) {
        int delta = steeringPID.output;

        if (steeringMode == STEERMODE_DECREASE) {
            // Depending on which way the bot is turning, choose which side to add correction to
            if (steeringPID.output <= 0) {
                //right = right + steeringPID.output;
                right = right + delta;
                if (right < 0) {
                    right = 0;
                } //clip right channel to zero
            } else //if(steeringPID.output > 0)
            {
                left = left - delta;
                //left = left - steeringPID.output;
                if (left < 0) {
                    left = 0;
                } //clip right channel to zero (one, actually)
            }
        } else if (steeringMode == STEERMODE_INCREASE) {
            // Depending on which way the bot is turning, choose which side to add correction to
            if (steeringPID.output <= 0) {
                //right = right + steeringPID.output;
                left = left - delta;
                if (right < 0) {
                    right = 0;
                } //clip right channel to zero
            } else //if(steeringPID.output > 0)
            {
                right = right + delta;
                //left = left - steeringPID.output;
                if (left < 0) {
                    left = 0;
                } //clip right channel to zero
            }
        } else if (steeringMode == STEERMODE_SPLIT) {
            right = right + delta / 2;
            left = left - delta / 2;
            if (right < 0) {
                left = left - right; //increase left, since right < 0
                right = 0;
            } //clip right channel to zero
            if (left < 0) {
                right = right - left; //increase right, since left < 0
                left = 0;
            } //clip left channel to zero
        }

    }//endif steeringPID.onoff

    outputs[0] = left;
    outputs[1] = right;

    //pidObjs[0].input = left;
    //pidObjs[1].input = right;
}

void steeringOff() {
    steeringPID.onoff = 0;
    steeringPID.p = 0;
    steeringPID.i = 0;
    steeringPID.d = 0;
    steeringPID.output = 0;
}

void steeringOn() {
    steeringPID.onoff = 1;
    steeringPID.p = 0;
    steeringPID.i = 0;
    steeringPID.d = 0;
    steeringPID.output = 0;
}

