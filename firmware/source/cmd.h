/******************************************************************************
* Name: cmd.h
* Desc: application specific command definitions are defined here.
* Date: 2010-07-10
* Author: stanbaek, apullin
Modifications and additions to this file made by Andrew Pullin are copyright, 2013
Copyrights are acknowledged for portions of this code extant before modifications by Andrew Pullin 
Any application of BSD or other license to copyright content without the authors express approval
is invalid and void.
******************************************************************************/
#ifndef __CMD_H
#define __CMD_H

#include "cmd_const.h"
#include <stdint.h>

//// Includes here should be to provide TYPES and ENUMS only
#include "move_queue.h"
#include "vr_leg_ctrl.h"


#define CMD_VECTOR_SIZE             0xFF //full length vector
#define MAX_CMD_FUNC                0x9F

#define CMD_SET_THRUST_OPENLOOP     0x80
#define CMD_SET_THRUST_CLOSEDLOOP   0x81
#define CMD_OR_SET_PID_GAINS        0x82
#define CMD_GET_PID_TELEMETRY       0x83
#define CMD_SET_CTRLD_TURN_RATE     0x84
#define CMD_STREAM_TELEMETRY        0x85
#define CMD_SET_MOVE_QUEUE	    0x86
#define CMD_SET_STEERING_GAINS      0x87
#define CMD_SOFTWARE_RESET          0x88
#define CMD_SPECIAL_TELEMETRY       0x89
#define CMD_ERASE_SECTORS           0x8A
#define CMD_FLASH_READBACK          0x8B
#define CMD_SLEEP                   0x8C
#define CMD_WHO_AM_I                0x8E
#define CMD_HALL_TELEMETRY          0x8F
#define CMD_ZERO_POS                0x90
#define CMD_SET_HALL_GAINS          0x91
#define CMD_SET_TAIL_QUEUE          0x92
#define CMD_SET_TAIL_GAINS          0x93
#define CMD_SET_THRUST_HALL         0x94
#define CMD_SET_OL_VIBE             0x95

//VelociRoACH specific functions
#define CMD_VR_SET_VEL_PROFILE      0x8D
#define CMD_VR_GET_AMS_POS          0x84
#define CMD_VR_START_TIMED_RUN      0x91
#define CMD_VR_START_TELEMETRY      0x8F
#define CMD_VR_SET_PID_GAINS        0x82
#define CMD_VR_SET_MOTOR_MODE       0x94
#define CMD_VR_PID_START_MOTORS     0x81
#define CMD_VR_PID_STOP_MOTORS      0x81
#define CMD_VR_SET_PHASE            0x93

unsigned int cmdSetup(void);
void cmdHandleRadioRxBuffer(void);


/////// Argument structures

//cmdSetThrustOpenLoop
typedef struct{
	int channel, dc;
} _args_cmdSetThrustOpenLoop;

//cmdSetMotorMode
typedef struct{
	int thrust1, thrust2;
} _args_cmdSetMotorMode;

//cmdSetThrustClosedLoop
typedef struct{
	int chan1;
        unsigned int runtime1;
        int chan2;
        unsigned int runtime2;
        unsigned int telem_samples;
} _args_cmdSetThrustClosedLoop;

//cmdORSetPIDGains
typedef struct{
	int Kp1, Ki1, Kd1, Kaw1, Kff1;
	int Kp2, Ki2, Kd2, Kaw2, Kff2;
} _args_cmdORSetPIDGains;

//cmdVRSetPIDGains
typedef struct{
	int Kp1, Ki1, Kd1, Kaw1, Kff1;
	int Kp2, Ki2, Kd2, Kaw2, Kff2;
} _args_cmdVRSetPIDGains;

//cmdSetCtrldTurnRate
typedef struct{
	int steerInput;
} _args_cmdSetCtrldTurnRate;

//cmdStreamTelemetry
typedef struct{
    unsigned long count;
} _args_cmdStreamTelemetry;

//cmdSetMoveQueue
//NOTE: This is not for the entire packet, just for one moveQ items,
// the cmd handler will stride across the packet, unpacking these
typedef struct{
	int inputL, inputR;
	unsigned long duration;
	enum moveSegT type;
	int params[3];
} _args_cmdSetMoveQueue;

//cmdSetSteeringGains
typedef struct{
	int Kp, Ki, Kd, Kaw, Kff;
        int steerMode;
} _args_cmdSetSteeringGains;

//cmdSoftwareReset
//no arguments

//cmdcmdStartTimedRun
typedef struct{
    uint16_t run_time;
} _args_cmdStartTimedRun;

//cmdStartTelemetry
typedef struct{
    uint32_t numSamples;
} _args_cmdStartTelemetry;

//cmdSpecialTelemetry
typedef struct{
	unsigned long count;
} _args_cmdSpecialTelemetry;

//cmdEraseSector
typedef struct{
	unsigned long samples;
} _args_cmdEraseSector;

//cmdFlashReadback
typedef struct{
	unsigned long samples;
} _args_cmdFlashReadback;

//cmdSleep

//cmdSetVelProfile
typedef struct{
    int16_t periodLeft;
    int16_t deltaL[NUM_VELS];
    int16_t flagLeft;
    int16_t periodRight;
    int16_t deltaR[NUM_VELS];
    int16_t flagRight;
} _args_cmdSetVelProfile;

//cmdHallTelemetry
typedef struct {
    unsigned long startDelay; // recording start time
    int count; // count of samples to record
    int skip; // samples to skip
} _args_cmdHallTelemetry;

//cmdSetTailQueue
//NOTE: This is not for the entire packet, just for one tailQ item,
// the cmd handler will stride across the packet, unpacking these
//typedef struct {
//    float angle;
//    unsigned long duration;
//    enum tailSegT type;
//    int params[3];
//} _args_cmdSetTailQueue;

//cmdSetTailGains
//typedef struct{
//	int Kp, Ki, Kd, Kaw, Kff;
//} _args_cmdSetTailGains;

//cmdSetThrustHall
typedef struct{
	int chan1;
        unsigned int runtime1;
        int chan2;
        unsigned int runtime2;
} _args_cmdSetThrustHall;

//cmdSetOLVibe
typedef struct{
	int channel;
        int incr;
        int amplitude;
        int phase;
} _args_cmdSetOLVibe;

//cmdSetPhase
typedef struct{
    int32_t offset;
} _args_cmdSetPhase;

#endif // __CMD_H

