// Authors: nkohut

#include "utils.h"
#include "pid.h"
#include "dfilter_avg.h"
#include "adc_pid.h"
#include "or_leg_ctrl.h"
#include "sys_service.h"
//#include "ams-enc.h"
#include "imu_service.h"

#if defined (__IMAGEPROC24)
    #include "gyro.h"
    #include "xl.h"
#elif defined (__IMAGEPROC25)
    #include "mpu6000.h"
#endif

#define TIMER_FREQUENCY     100.0                 // 300 Hz
#define TIMER_PERIOD        1/TIMER_FREQUENCY   //This is used for numerical integration

//Setup for Gyro Z averaging filter
#define GYRO_AVG_SAMPLES 	4

#define ABS(my_val) ((my_val) < 0) ? -(my_val) : (my_val)

//Filter stuctures for gyro variables
static dfilterAvgInt_t gyroZavg;


//TODO: change these to arrays
static int lastGyro[3] = {0,0,0};
static int lastXL[3] = {0,0,0};

static float lastGyroDeg[3] = {0,0,0};

static int lastGyroZValueAvg = 0;

static float lastGyroZValueAvgDeg = 0.0;

static float lastBodyZPositionDeg = 0.0;

static void SetupTimer4(); 
static void imuServiceRoutine(void);  //To be installed with sysService
//The following local functions are called by the service routine:
static void imuISRHandler(void);

////   Private functions
////////////////////////
/////////        IMU ISR          ////////
////////  Installed to Timer4 @ 300hz  ////////

static void imuServiceRoutine(void){
    //This intermediate function is used in case we want to tie other
    //sub-taks to the imu service routine.
    //TODO: Is this neccesary?
    imuISRHandler();
}

static void imuISRHandler() {

    LED_GREEN = 1;

    int gyroData[3];
    int xlData[3];

#if defined (__IMAGEPROC24)
            /////// Get Gyro data and calc average via filter
            gyroReadXYZ(); //bad design of gyro module; todo: humhu
    gyroGetIntXYZ(gyroData);
#elif defined (__IMAGEPROC25)
    mpuBeginUpdate();
    mpuGetGyro(gyroData);
    mpuGetXl(xlData);
#endif


    lastGyro[0] = gyroData[0];
    lastGyro[1] = gyroData[1];
    lastGyro[2] = gyroData[2];

    lastXL[0] = xlData[0];
    lastXL[1] = xlData[1];
    lastXL[2] = xlData[2];


    int i;
    for (i = 0; i < 3; i++) {
        //Threshold:
        if (ABS(lastGyro[i]) < GYRO_DRIFT_THRESH) {
            lastGyro[0] = lastGyro[i] >> 1; //fast divide by 2
        }
    }


    lastGyroDeg[0] = (float) (lastGyro[0] * LSB2DEG);
    lastGyroDeg[1] = (float) (lastGyro[1] * LSB2DEG);
    lastGyroDeg[2] = (float) (lastGyro[2] * LSB2DEG);

    dfilterAvgUpdate(&gyroZavg, gyroData[2]);

    lastGyroZValueAvg = dfilterAvgCalc(&gyroZavg);

    lastGyroZValueAvgDeg = (float) lastGyroZValueAvg*LSB2DEG;

    lastBodyZPositionDeg = lastBodyZPositionDeg + lastGyroDeg[2]*TIMER_PERIOD;

    LED_GREEN = 0;
}

static void SetupTimer4(){
    ///// Timer 4 setup, 300Hz /////
    // period value = Fcy/(prescale*Ftimer)
    unsigned int T4CON1value, T4PERvalue;
    // prescale 1:64
    T4CON1value = T4_ON & T4_IDLE_CON & T4_GATE_OFF & T4_PS_1_64 & T4_SOURCE_INT;
    // Period is set so that period = 3.3ms (300Hz), MIPS = 40
    //T4PERvalue = 2083; // ~300Hz (40e6/(64*2083) where 64 is the prescaler
    T4PERvalue = 625;  // 1Khz
    int retval;
    retval = sysServiceConfigT4(T4CON1value, T4PERvalue, T4_INT_PRIOR_3 & T4_INT_ON);
}

////   Public functions
////////////////////////
void imuSetup(){
    int retval;
    retval = sysServiceInstallT4(imuServiceRoutine);
    SetupTimer4();
    dfilterAvgCreate(&gyroZavg, GYRO_AVG_SAMPLES);
}

int imuGetGyroXValue() {
    return lastGyro[0];
}

int imuGetGyroYValue() {
    return lastGyro[1];
}

int imuGetGyroZValue() {
    return lastGyro[2];
}


float imuGetGyroXValueDeg() {
    return lastGyroDeg[0];
}

float imuGetGyroYValueDeg() {
    return lastGyroDeg[1];
}

float imuGetGyroZValueDeg() {
    return lastGyroDeg[2];
}


int imuGetGyroZValueAvg() {
    return lastGyroZValueAvg;
}

float imuGetGyroZValueAvgDeg() {
    return lastGyroZValueAvgDeg;
}


float imuGetBodyZPositionDeg() {
    return lastBodyZPositionDeg;
}

void imuResetGyroZAvg(){
    dfilterZero(&gyroZavg);
}

int imuGetXLXValue() {
    return lastXL[0];
}

int imuGetXLYValue() {
    return lastXL[1];
}

int imuGetXLZValue() {
    return lastXL[2];
}