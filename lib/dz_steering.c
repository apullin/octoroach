#include "settings.h"

#include "dz_steering.h"
#include "timer.h"
#include "gyro.h"
#include "stopwatch.h"
#include "dfmem.h"
#include "move_queue.h"
#include "dfilter_avg.h"
#include "leg_ctrl.h"
#include "sys_service.h"
#include "hall_queue.h"

//Inline functions
#define ABS(a)	   (((a) < 0) ? -(a) : (a))

//Averaging filter structures for gyroscope data
//Initialzied in setup.
filterAvgInt_t dz_gyroZavg; //This is exported for use in the telemetry module
#define DZ_GYRO_AVG_SAMPLES 	32

#define DZ_GYRO_DRIFT_THRESH 5

extern moveCmdT currentMove, idleMove;

hallVelLUT leftTurnLUT, rightTurnLUT, straightLUT;

int dz_steeringOnOff = DZ_STEERING_ON;
int dz_steering_angle = 0;

//Function to be installed into T5, and setup function
static void SetupTimer5();
static void dz_steeringServiceRoutine(void);  //To be installed with sysService
//The following local functions are called by the service routine:
static void dz_steeringHandleISR();


////   Private functions
////////////////////////

/////////        Steering ISR          ////////
////////  Installed to Timer5 @ 300hz  ////////
//void __attribute__((interrupt, no_auto_psv)) _T5Interrupt(void) {
static void dz_steeringServiceRoutine(void){
    //This intermediate function is used in case we want to tie other
    //sub-taks to the steering service routine.
    //TODO: Is this neccesary?

    // Steering update ISR handler
    dz_steeringHandleISR();
}

static void SetupTimer5(){
    ///// Timer 5 setup, Steering ISR, 300Hz /////
    // period value = Fcy/(prescale*Ftimer)
    unsigned int T5CON1value, T5PERvalue;
    // prescale 1:64
    T5CON1value = T5_ON & T5_IDLE_CON & T5_GATE_OFF & T5_PS_1_64 & T5_SOURCE_INT;
    // Period is set so that period = 5ms (200Hz), MIPS = 40
    //period = 3125; // 200Hz
    T5PERvalue = 2083; // ~300Hz
    int retval;
    retval = sysServiceConfigT5(T5CON1value, T5PERvalue, T5_INT_PRIOR_5 & T5_INT_ON);
}



////   Public functions
////////////////////////

void dz_steeringSetup(void) {

    dz_steeringSetAngRate(0);

    SetupTimer5(); //T5 ISR will update the steering controller
    int retval;
    retval = sysServiceInstallT5(dz_steeringServiceRoutine);

    //Averaging filter setup:
    filterAvgCreate(&dz_gyroZavg, DZ_GYRO_AVG_SAMPLES);
}

void dz_steeringSetAngRate(int angRate) {
    dz_steering_angle = angRate;
}

static void dz_steeringHandleISR() {

    //int gyroAvg[3];
    int wz;
    int gyroData[3];
    int gyroOffsets[3];

    gyroGetXYZ((unsigned char*) gyroData);
    gyroGetOffsets(gyroOffsets);

    filterAvgUpdate(&dz_gyroZavg, gyroData[2] - gyroOffsets[2]);

    wz = filterAvgCalc(&dz_gyroZavg);

    //Threshold filter on gyro to account for minor drift
    //if (ABS(wz) < GYRO_DRIFT_THRESH) {
    //    wz = 0;
    //}

    //Update the setpoints
    //if((currentMove->inputL != 0) && (currentMove->inputR != 0)){
    if (currentMove != idleMove) {
        //if wz < -deadband , set leftTurnLUT
        //if wz > deadband , set rightTurnLUT
        //else set straightTurnLUT
    }
    //Now the output correction is stored in steeringPID.output,
    //which will be read later when the steering mixing is done.
}

void steeringOff() {
    dz_steeringOnOff = DZ_STEERING_ON;
}

void steeringOn() {
    dz_steeringOnOff = DZ_STEERING_OFF;
}

