/***************************************************************************
 * Name: cmd.c
 * Desc: Receiving and transmitting queue handler
 * Date: 2010-07-10
 * Author: stanbaek, apullin
Modifications and additions to this file made by Andrew Pullin are copyright, 2013
Copyrights are acknowledged for portions of this code extant before modifications by Andrew Pullin 
Any application of BSD or other license to copyright content without the authors express approval
is invalid and void.
 **************************************************************************/

#include "cmd.h"
#include "cmd_const.h"
#include "dfmem.h"
#include "utils.h"
#include "ports.h"
#include "gyro.h"
#include "xl.h"
#include "sclock.h"
#include "motor_ctrl.h"
//#include "sensors.h"
#include "dfmem.h"
#include "pid.h"
#include "radio.h"
#include "payload.h"
#include "move_queue.h"
#include "steering.h"
#include "telem_service.h"
#include "or_leg_ctrl.h"
#include "tail_ctrl.h"
//#include "hall.h"
#include "version.h"
#include "tih.h"
#include "ol-vibe.h"
#include "vr_leg_ctrl.h"

#include "settings.h" //major config defines, sys-service, hall, etc

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define PKT_UNPACK(type, var, pktframe) type* var = (type*)(pktframe);

unsigned char tx_frame_[127];

extern MoveQueue moveq;
//extern TailQueue tailq;

//extern volatile char g_radio_duty_cycle;

#define _cmdSetupHandler(CODE, funcPointer)  cmd_func[CODE] = &funcPointer;

// use an array of function pointer to avoid a number of case statements
// CMD_VECTOR_SIZE is defined in cmd_const.h
static void (*cmd_func[CMD_VECTOR_SIZE])(unsigned char, unsigned char, unsigned char*, unsigned int);

/*-----------------------------------------------------------------------------
 *          Declaration of static functions
-----------------------------------------------------------------------------*/

// System functions
static void cmdEraseMemSector(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdEcho(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdNop(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);

//User commands
static void cmdSetThrustOpenLoop(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdSetThrustClosedLoop(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdORSetPIDGains(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdSetCtrldTurnRate(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdSetMoveQueue(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdSetSteeringGains(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdSoftwareReset(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdSpecialTelemetry(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdEraseSector(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);  //TODO: Needs to be renamed
static void cmdFlashReadback(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdSleep(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdWhoAmI(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdZeroPos(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdSetOLVibe(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);

static void cmdVRSetVelProfile(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdVRGetAMSPos(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdVRSetMotorMode(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdVRPIDStartMotors(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdVRPIDStopMotors(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdVRSetPIDGains(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdVRSetPhase(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdVRStartTimedRun(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdVRStartTelemetry(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);

//vestigial
//static void cmdGetPIDTelemetry(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
//static void cmdHallTelemetry(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
//static void cmdSetThrustHall(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);

/*-----------------------------------------------------------------------------
 *          Public functions
-----------------------------------------------------------------------------*/
unsigned int cmdSetup(void) {

    unsigned int i;

    // initialize the array of func pointers with Nop()
    for (i = 0; i < MAX_CMD_FUNC; ++i) {
        cmd_func[i] = &cmdNop;
        //_cmdSetupHandler(i, cmdNop);
        //cmd_len[i] = 0; //0 indicated an unpoplulated command
    }

    //System commands
    _cmdSetupHandler(CMD_ECHO, cmdEcho);
    _cmdSetupHandler(CMD_ERASE_MEM_SECTOR, cmdEraseMemSector);

    //User commands
    _cmdSetupHandler(CMD_SET_THRUST_OPENLOOP,   cmdSetThrustOpenLoop);
    _cmdSetupHandler(CMD_SET_THRUST_CLOSEDLOOP, cmdSetThrustClosedLoop);
    _cmdSetupHandler(CMD_OR_SET_PID_GAINS,      cmdORSetPIDGains);
    _cmdSetupHandler(CMD_SET_CTRLD_TURN_RATE,   cmdSetCtrldTurnRate);
    _cmdSetupHandler(CMD_SET_MOVE_QUEUE,        cmdSetMoveQueue);
    _cmdSetupHandler(CMD_SET_STEERING_GAINS,    cmdSetSteeringGains);
    _cmdSetupHandler(CMD_SOFTWARE_RESET,        cmdSoftwareReset);
    _cmdSetupHandler(CMD_SPECIAL_TELEMETRY,     cmdSpecialTelemetry);
    _cmdSetupHandler(CMD_ERASE_SECTORS,         cmdEraseSector);
    _cmdSetupHandler(CMD_FLASH_READBACK,        cmdFlashReadback);
    _cmdSetupHandler(CMD_SLEEP,                 cmdSleep);
    _cmdSetupHandler(CMD_WHO_AM_I,              cmdWhoAmI);
    _cmdSetupHandler(CMD_ZERO_POS,              cmdZeroPos);
    _cmdSetupHandler(CMD_SET_OL_VIBE,           cmdSetOLVibe);

    //VelociRoACH / vr_leg_ctrl functions
    _cmdSetupHandler(CMD_VR_SET_VEL_PROFILE,    cmdVRSetVelProfile);
    _cmdSetupHandler(CMD_VR_GET_AMS_POS,        cmdVRGetAMSPos);
    _cmdSetupHandler(CMD_VR_START_TIMED_RUN,    cmdVRStartTimedRun);
    _cmdSetupHandler(CMD_VR_START_TELEMETRY,    cmdVRStartTelemetry);
    _cmdSetupHandler(CMD_VR_SET_PID_GAINS,      cmdVRSetPIDGains);
    _cmdSetupHandler(CMD_VR_SET_MOTOR_MODE,     cmdVRSetMotorMode);
    _cmdSetupHandler(CMD_VR_PID_START_MOTORS,   cmdVRPIDStartMotors);
    _cmdSetupHandler(CMD_VR_PID_STOP_MOTORS,    cmdVRPIDStopMotors);
    _cmdSetupHandler(CMD_VR_PID_STOP_MOTORS,    cmdVRPIDStopMotors);
    _cmdSetupHandler(CMD_VR_SET_PHASE,          cmdVRSetPhase);

    //////vestigial
    //_cmdSetupHandler(CMD_STREAM_TELEMETRY, cmdGetImuLoopZGyro);
    //_cmdSetupHandler(CMD_GET_PID_TELEMETRY, cmdGetPIDTelemetry);
    //_cmdSetupHandler(CMD_HALL_TELEMETRY, cmdHallTelemetry);
    //_cmdSetupHandler(CMD_SET_HALL_GAINS, cmdSetHallGains);
    //_cmdSetupHandler(CMD_SET_TAIL_QUEUE, cmdSetTailQueue);
    //_cmdSetupHandler(CMD_SET_TAIL_GAINS, cmdSetTailGains);
    //_cmdSetupHandler(CMD_SET_THRUST_HALL, cmdSetThrustHall);
    
    return 1;
}

void cmdHandleRadioRxBuffer(void) {
    MacPacket packet;
    Payload pld;
    unsigned char command, status;
    
    if ((packet = radioDequeueRxPacket()) != NULL) {
        //LED_YELLOW = 1;
        pld = macGetPayload(packet);
        status = payGetStatus(pld);
        command = payGetType(pld);
        unsigned int rx_src_addr = packet->src_addr.val;

        if (command < MAX_CMD_FUNC) {
            cmd_func[command](status, payGetDataLength(pld), payGetData(pld), rx_src_addr);
        }
        radioReturnPacket(packet);
    }
    
    return;
}


/*-----------------------------------------------------------------------------
 *          IMU functions
-----------------------------------------------------------------------------*/

static void cmdEraseMemSector(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    unsigned int page;
    page = frame[0] + (frame[1] << 8);
    LED_RED = 1;
    dfmemEraseSector(0x0100); // erase Sector 1 (page 256 - 511)
    LED_RED = 0;
}

/*-----------------------------------------------------------------------------
 *          AUX functions
-----------------------------------------------------------------------------*/
void cmdEcho(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    //Note that the destination is the hard-coded RADIO_DST_ADDR
    //todo : extract the destination address properly.
    radioSendData(src_addr, 0, CMD_ECHO, length, frame, 0);
}

static void cmdNop(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    Nop();
}

/*-----------------------------------------------------------------------------
 *         User function
-----------------------------------------------------------------------------*/
static void cmdSetThrustOpenLoop(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    //Unpack unsigned char* frame into structured values
    PKT_UNPACK(_args_cmdSetThrustOpenLoop, argsPtr, frame);

    //set motor duty cycle
    tiHSetDC(argsPtr->channel, argsPtr->dc);
}

static void cmdSetThrustClosedLoop(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    //Unpack unsigned char* frame into structured values
    PKT_UNPACK(_args_cmdSetThrustClosedLoop, argsPtr, frame);

    orLegCtrlSetInput(LEG_CTRL_LEFT, argsPtr->chan1);
    orLegCtrlOnOff(LEG_CTRL_LEFT, PID_ON); //Motor PID #1 -> ON

    orLegCtrlSetInput(LEG_CTRL_RIGHT, argsPtr->chan2);
    orLegCtrlOnOff(LEG_CTRL_RIGHT, PID_ON); //Motor PID #2 -> ON
}

static void cmdORSetPIDGains(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    //Unpack unsigned char* frame into structured values
    PKT_UNPACK(_args_cmdORSetPIDGains, argsPtr, frame);

    orLegCtrlSetGains(OCTOROACH_LEFT_LEGS_PID_NUM,  argsPtr->Kp1, argsPtr->Ki1, argsPtr->Kd1, argsPtr->Kaw1, argsPtr->Kff1);
    orLegCtrlSetGains(OCTOROACH_RIGHT_LEGS_PID_NUM, argsPtr->Kp2, argsPtr->Ki2, argsPtr->Kd2, argsPtr->Kaw2, argsPtr->Kff2);

    //Send confirmation packet, which is the exact same data payload as what was sent
    //Note that the destination is the hard-coded RADIO_DST_ADDR
    //todo : extract the destination address properly.
    radioSendData(src_addr, 0, CMD_OR_SET_PID_GAINS, length, frame, 0);

}

static void cmdSetCtrldTurnRate(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    //Unpack unsigned char* frame into structured values
    PKT_UNPACK(_args_cmdSetCtrldTurnRate, argsPtr, frame);
    steeringSetInput(argsPtr->steerInput);

    //Send confirmation packet, which is the exact same data payload as what was sent
    //Note that the destination is the hard-coded RADIO_DST_ADDR
    //todo : extract the destination address properly.
    radioSendData(src_addr, 0, CMD_SET_CTRLD_TURN_RATE, length, frame, 0);
}

static void cmdSetMoveQueue(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    //The count is read via standard pointer math
    unsigned int count;
    count = (unsigned int) (*(frame));

    //Unpack unsigned char* frame into structured values
    //Here, unpacking happens in the loop below.
    //Due to variable length, PKT_UNPACK is not used here
    int idx = sizeof (count); //should be an unsigned int, sizeof(uint) = 2

    moveCmdT move;
    int i;
    for (i = 0; i < count; i++) {
        move = (moveCmdT) malloc(sizeof (moveCmdStruct));
        //argsPtr = (_args_cmdSetMoveQueue*)(frame+idx);
        *move = *((moveCmdT) (frame + idx));
        mqPush(moveq, move);
        //idx =+ sizeof(_args_cmdSetMoveQueue);
        idx += sizeof (moveCmdStruct);
    }
}

//Format for steering gains:
// [Kp, Ki, Kd, Kaw, Kff, steeringMode]
//

static void cmdSetSteeringGains(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    //Unpack unsigned char* frame into structured values
    PKT_UNPACK(_args_cmdSetSteeringGains, argsPtr, frame);

    steeringSetGains(argsPtr->Kp, argsPtr->Ki, argsPtr->Kd, argsPtr->Kaw, argsPtr->Kff);
    steeringSetMode(argsPtr->steerMode);

    //Send confirmation packet, which is the exact same data payload as what was sent
    //Note that the destination is the hard-coded RADIO_DST_ADDR
    //todo : extract the destination address properly.
    radioSendData(src_addr, 0, CMD_SET_STEERING_GAINS, length, frame, 1);
}

static void cmdSoftwareReset(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    char resetmsg[] = "RESET";
    int len = strlen(resetmsg);

    MacPacket response;
    response = radioRequestPacket(len);

    macSetDestPan(response, RADIO_PAN_ID);
    macSetDestAddr(response, src_addr);
    Payload pld = macGetPayload(response);

    paySetData(pld, len, (unsigned char*)resetmsg);
    paySetType(pld, CMD_SOFTWARE_RESET);
    paySetStatus(pld, 0);
    
    while(!radioEnqueueTxPacket(response)) { radioProcess(); }

#ifndef __DEBUG
    __asm__ volatile ("reset");
#endif
}

static void cmdSpecialTelemetry(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    //Unpack unsigned char* frame into structured values
    PKT_UNPACK(_args_cmdSpecialTelemetry, argsPtr, frame);

    if (argsPtr->count != 0) {
        telemSetStartTime(); // Start telemetry samples from approx 0 time
        telemSetSamplesToSave(argsPtr->count);
    }
}

static void cmdEraseSector(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    //Unpack unsigned char* frame into structured values
    PKT_UNPACK(_args_cmdEraseSector, argsPtr, frame);

    telemErase(argsPtr->samples);

    //Send confirmation packet; this only happens when flash erase is completed.
    //Note that the destination is the hard-coded RADIO_DST_ADDR
    //todo : extract the destination address properly.
    radioSendData(src_addr, 0, CMD_ERASE_SECTORS, length, frame, 0);
}

static void cmdFlashReadback(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    //LED_YELLOW = 1;
    PKT_UNPACK(_args_cmdFlashReadback, argsPtr, frame);

    //Horibble hack: Disable IMU while erading back telem
    //_T4IE = 0; // only for 'roach' project
    imuServiceDisable();
    
    telemReadbackSamples(argsPtr->samples, src_addr);

    //Horibble hack: Disable IMU while erading back telem
    //_T4IE = 1; // only for 'roach' project
    imuServiceEnable();
}

static void cmdSleep(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    //Currently unused
    //todo : review power reduction organization
    /*char sleep = frame[0];
    if (sleep) {
        //g_radio_duty_cycle = 1;
    } else {
        //g_radio_duty_cycle = 0;
        Payload pld;
        pld = payCreateEmpty(1);
        paySetData(pld, 1, (unsigned char*) (&sleep)); //echo back a CMD_SLEEP with '0', incdicating a wakeup
        paySetStatus(pld, status);
        paySetType(pld, CMD_SLEEP);
        radioSendPayload(RADIO_DST_ADDR, pld);
    }*/
}

// set up velocity profile structure  - assume 4 set points for now, generalize later
static void cmdVRSetVelProfile(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {

    //Unpack unsigned char* frame into structured values
    PKT_UNPACK(_args_cmdSetVelProfile, argsPtr, frame);

    int interval1[NUM_VELS], vel1[NUM_VELS], delta1[NUM_VELS];
    int interval2[NUM_VELS], vel2[NUM_VELS], delta2[NUM_VELS];
    int i;

    for(i = 0; i < NUM_VELS; i++){
        delta1[i] = argsPtr->deltaL[i] << 2;//cmdSetVelProfile
        delta2[i] = argsPtr->deltaR[i] << 2;
        //Clipping of deltas to range [-8192, 8191] ?

        // Calculation of intervals is fixed to equally spaced intervals.
        interval1[i] = argsPtr->periodLeft/NUM_VELS;
        interval2[i] = argsPtr->periodRight/NUM_VELS;

        vel1[i] = delta1[i] / interval1[i];
        vel2[i] = delta2[i] / interval2[i];
    }

    pidip25setPIDVelProfile(LEFT_LEGS_PID_NUM, interval1, delta1, vel1, argsPtr->flagLeft);
    pidip25setPIDVelProfile(RIGHT_LEGS_PID_NUM, interval2, delta2, vel2, argsPtr->flagRight);

    //Send confirmation packet
    // TODO : Send confirmation packet with packet index
    //return 1; //success
}

// report motor position and  reset motor position (from Hall effect sensors)
// note motor_count is long (4 bytes)
static void cmdZeroPos(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
/*
    hallZeroPos(0);
    hallZeroPos(1);

    // This function in unsafe. Passing pointers between modules is disallowed.
    long* hallCounts = hallGetMotorCounts();

    //Note that the destination is the hard-coded RADIO_DST_ADDR
    //todo : extract the destination address properly.
    radioSendData(RADIO_DST_ADDR, 0, CMD_ZERO_POS, 2*sizeof(unsigned long), (unsigned char*)hallCounts, 0);
 */
}

// send robot info when queried
static void cmdWhoAmI(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    // maximum string length to avoid packet size limit
    char* verstr = versionGetString();
    int verlen = strlen(verstr);

    //Note that the destination is the hard-coded RADIO_DST_ADDR
    //todo : extract the destination address properly.
    radioSendData(src_addr, 0, CMD_WHO_AM_I, verlen, (unsigned char*)verstr, 0);
}

static void cmdSetOLVibe(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    //Unpack unsigned char* frame into structured values
    PKT_UNPACK(_args_cmdSetOLVibe, argsPtr, frame);

    olVibeSetFrequency(argsPtr->channel, argsPtr->incr);
    olVibeSetPhase(argsPtr->channel, argsPtr->phase);
    olVibeSetAmplitude(argsPtr->channel, argsPtr->amplitude);
}

static void cmdVRSetMotorMode(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    //Unpack unsigned char* frame into structured values
    PKT_UNPACK(_args_cmdSetMotorMode, argsPtr, frame);


    pidip25SetPWMDes(LEFT_LEGS_PID_NUM, argsPtr->thrust1);
    pidip25SetPWMDes(RIGHT_LEGS_PID_NUM, argsPtr->thrust2);

    pidip25SetMode(LEFT_LEGS_PID_NUM,1);

    //return 1;
 }

static void cmdVRGetAMSPos(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    long motor_count[2];
    motor_count[0] = pidip25GetPState(LEFT_LEGS_PID_NUM);
    motor_count[1] = pidip25GetPState(RIGHT_LEGS_PID_NUM);

    radioSendData(src_addr, status, CMD_VR_GET_AMS_POS,  //TODO: Robot should respond to source of query, not hardcoded address
            sizeof(motor_count), (unsigned char *)motor_count, 0);

    //return 1;
}

static void cmdVRPIDStartMotors(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {

    //All actions have been moved to a PID module function
    pidip25StartMotor(LEFT_LEGS_PID_NUM);
    pidip25StartMotor(RIGHT_LEGS_PID_NUM);

    //return 1;
}

static void cmdVRPIDStopMotors(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {

    pidip25Off(LEFT_LEGS_PID_NUM);
    pidip25Off(RIGHT_LEGS_PID_NUM);

    //return 1;
}

static void cmdVRSetPhase(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    //Unpack unsigned char* frame into structured values
    PKT_UNPACK(_args_cmdSetPhase, argsPtr, frame);

    long p_state[2], error;
    p_state[0] = pidip25GetPState(LEFT_LEGS_PID_NUM);
    p_state[1] = pidip25GetPState(RIGHT_LEGS_PID_NUM);

    error = argsPtr->offset - ( (p_state[0] & 0x0000FFFF) - (p_state[1] & 0x0000FFFF) );

    pidip25SetPInput(LEFT_LEGS_PID_NUM, p_state[0] + error/2);
    pidip25SetPInput(RIGHT_LEGS_PID_NUM, p_state[1] - error/2);

    //return 1;
}

static void cmdVRSetPIDGains(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    //Unpack unsigned char* frame into structured values
    PKT_UNPACK(_args_cmdVRSetPIDGains, argsPtr, frame);
    pidip25SetGains(LEFT_LEGS_PID_NUM,
            argsPtr->Kp1,argsPtr->Ki1,argsPtr->Kd1,argsPtr->Kaw1, argsPtr->Kff1);

    pidip25SetGains(RIGHT_LEGS_PID_NUM,
            argsPtr->Kp2,argsPtr->Ki2,argsPtr->Kd2,argsPtr->Kaw2, argsPtr->Kff2);

    radioSendData(src_addr, status, CMD_VR_SET_PID_GAINS, length, frame, 0); //TODO: Robot should respond to source of query, not hardcoded address

    //return 1; //success
}

static void cmdVRStartTimedRun(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    //Unpack unsigned char* frame into structured values
    PKT_UNPACK(_args_cmdStartTimedRun, argsPtr, frame);

    int i;
    for (i = 0; i < NUM_PIDS; i++){
        pidip25SetTimeFlag(i,1);
        //pidObjs[i].timeFlag = 1;
        pidip25SetInput(i, 0);
        checkSwapBuff(i);
        pidip25On(i);
    }

    pidip25SetMode(LEFT_LEGS_PID_NUM ,PID_MODE_CONTROLED);
    pidip25SetMode(RIGHT_LEGS_PID_NUM ,PID_MODE_CONTROLED);

    pidip25StartTimedTrial(argsPtr->run_time);

    //return 1;
}

static void cmdVRStartTelemetry(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    //Unpack unsigned char* frame into structured values
    PKT_UNPACK(_args_cmdStartTelemetry, argsPtr, frame);

    if (argsPtr->numSamples != 0) {
        telemSetStartTime(); // Start telemetry samples from approx 0 time
        telemSetSamplesToSave(argsPtr->numSamples);
    }
    //return 1;
}