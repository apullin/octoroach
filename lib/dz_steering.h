#ifndef __DZ_STEERING_H
#define __DZ_STEERING_H

#include "pid.h"

void dz_steeringSetup(void);
void dz_steeringSetAngRate(int angRate);
void dz_steeringSetGains(int Kp,int Ki,int Kd,int Kawm, int ff);
void dz_steeringSetMode(unsigned int mode);
void dz_steeringApplyCorrection(int* inputs, int* outputs);
void dz_steeringOff();
void dz_steeringOn();


#define DZ_STEERING_OFF 0
#define DZ_STEERING_ON  1

/*
enum STEERING_MODES { 
	STEERMODE_INCREASE = 1,
	STEERMODE_DECREASE = 0,
	STEERMODE_SPLIT	   = 2
};
*/

#endif //__DZ_STEERING_H
