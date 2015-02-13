// Contents of this file are copyright Andrew Pullin, 2013

//or_telem.c , OctoRoACH specific telemetry packet format


#include "or_telem.h"

#include <xc.h>
#include "pid.h"
#include "gyro.h"
#include "xl.h"
#include "ams-enc.h"
#include "imu.h"
#include "leg_ctrl.h"
#include "tail_ctrl.h"
#include "adc_pid.h"
#include "tail_ctrl.h"
#include "tih.h"
#include "steering.h"

// TODO (apullin) : Remove externs by adding getters to other modules
//extern pidObj motor_pidObjs[NUM_MOTOR_PIDS];
//extern int bemf[NUM_MOTOR_PIDS];
//extern pidObj steeringPID;
//extern pidObj tailPID;

void orTelemGetData(unsigned char* ptr) {
    /////// Get XL data
    orTelemStruct_t* tptr;
    tptr = (orTelemStruct_t*) ptr;
 
    tptr->inputL = legCtrlGetInput(1);
    tptr->inputR = legCtrlGetInput(2);
    tptr->dcA = tiHGetSignedDC(1);
    tptr->dcB = tiHGetSignedDC(2);
    tptr->dcC = tiHGetSignedDC(3);
    tptr->dcD = tiHGetSignedDC(4);
    tptr->gyroX = imuGetGyroXValue();
    tptr->gyroY = imuGetGyroYValue();
    tptr->gyroZ = imuGetGyroZValue();
    tptr->gyroAvg =imuGetGyroZValueAvgDeg();
    tptr->accelX = imuGetXLXValue();
    tptr->accelY = imuGetXLYValue();
    tptr->accelZ = imuGetXLZValue();
    tptr->bemfA = adcGetMotorA();
    tptr->bemfB = adcGetMotorB();
    tptr->bemfC = adcGetMotorC();
    tptr->bemfD = adcGetMotorD();
    tptr->steerIn = steeringGetInput();
    tptr->steerOut = steeringGetInput();
    tptr->Vbatt = adcGetVbatt();
    tptr->yawAngle = imuGetBodyZPositionDeg();

}

//This may be unneccesary, since the telemtry type isn't totally anonymous

unsigned int orTelemGetSize() {
    return sizeof (orTelemStruct_t);
}