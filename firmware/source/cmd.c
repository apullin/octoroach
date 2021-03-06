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
#include "leg_ctrl.h"
#include "tail_ctrl.h"
//#include "hall.h"
#include "version.h"
#include "tih.h"
#include "ol-vibe.h"

#include "settings.h" //major config defines, sys-service, hall, etc

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define PKT_UNPACK(type, var, pktframe) type* var = (type*)(pktframe);

unsigned char tx_frame_[127];

extern MoveQueue moveq;
extern TailQueue tailq;

//extern volatile char g_radio_duty_cycle;

#define _cmdSetupHandler(CODE, funcPointer)  cmd_func[CODE] = &funcPointer;

// use an array of function pointer to avoid a number of case statements
// CMD_VECTOR_SIZE is defined in cmd_const.h
void (*cmd_func[CMD_VECTOR_SIZE])(unsigned char, unsigned char, unsigned char*, unsigned int);

/*-----------------------------------------------------------------------------
 *          Declaration of static functions
-----------------------------------------------------------------------------*/
static void cmdSetThrust(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdSteer(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);

static void cmdEraseMemSector(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);

static void cmdEcho(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);

static void cmdNop(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);

//User commands
static void cmdSetThrustOpenLoop(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdSetThrustClosedLoop(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdSetPIDGains(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdGetPIDTelemetry(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdSetCtrldTurnRate(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdGetImuLoopZGyro(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdSetMoveQueue(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdSetSteeringGains(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdSoftwareReset(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdSpecialTelemetry(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdEraseSector(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdFlashReadback(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdSleep(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdSetVelProfile(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdWhoAmI(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdHallTelemetry(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdZeroPos(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdSetHallGains(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdSetTailQueue(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdSetTailGains(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdSetThrustHall(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static void cmdSetOLVibe(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);

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

    _cmdSetupHandler(CMD_ECHO, cmdEcho);
    _cmdSetupHandler(CMD_SET_THRUST, cmdSetThrust);
    _cmdSetupHandler(CMD_SET_STEER, cmdSteer);
    _cmdSetupHandler(CMD_ERASE_MEM_SECTOR, cmdEraseMemSector);
    //User commands
    _cmdSetupHandler(CMD_SET_THRUST_OPENLOOP, cmdSetThrustOpenLoop);
    _cmdSetupHandler(CMD_SET_THRUST_CLOSEDLOOP, cmdSetThrustClosedLoop);
    _cmdSetupHandler(CMD_SET_PID_GAINS, cmdSetPIDGains);
    _cmdSetupHandler(CMD_GET_PID_TELEMETRY, cmdGetPIDTelemetry);
    _cmdSetupHandler(CMD_SET_CTRLD_TURN_RATE, cmdSetCtrldTurnRate);
    _cmdSetupHandler(CMD_STREAM_TELEMETRY, cmdGetImuLoopZGyro);
    _cmdSetupHandler(CMD_SET_MOVE_QUEUE, cmdSetMoveQueue);
    _cmdSetupHandler(CMD_SET_STEERING_GAINS, cmdSetSteeringGains);
    _cmdSetupHandler(CMD_SOFTWARE_RESET, cmdSoftwareReset);
    _cmdSetupHandler(CMD_SPECIAL_TELEMETRY, cmdSpecialTelemetry);
    _cmdSetupHandler(CMD_ERASE_SECTORS, cmdEraseSector);
    _cmdSetupHandler(CMD_FLASH_READBACK, cmdFlashReadback);
    _cmdSetupHandler(CMD_SLEEP, cmdSleep);
    _cmdSetupHandler(CMD_SET_VEL_PROFILE, cmdSetVelProfile);
    _cmdSetupHandler(CMD_WHO_AM_I, cmdWhoAmI);
    _cmdSetupHandler(CMD_HALL_TELEMETRY, cmdHallTelemetry);
    _cmdSetupHandler(CMD_ZERO_POS, cmdZeroPos);
    _cmdSetupHandler(CMD_SET_HALL_GAINS, cmdSetHallGains);
    _cmdSetupHandler(CMD_SET_TAIL_QUEUE, cmdSetTailQueue);
    _cmdSetupHandler(CMD_SET_TAIL_GAINS, cmdSetTailGains);
    _cmdSetupHandler(CMD_SET_THRUST_HALL, cmdSetThrustHall);
    _cmdSetupHandler(CMD_SET_OL_VIBE, cmdSetOLVibe);
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
 * ----------------------------------------------------------------------------
 * The functions below are intended for internal use, i.e., private methods.
 * Users are recommended to use functions defined above.
 * ----------------------------------------------------------------------------
-----------------------------------------------------------------------------*/

static void cmdSetThrust(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
/*
    unsigned char chr_test[4];
    float *duty_cycle = (float*) chr_test;

    chr_test[0] = frame[0];
    chr_test[1] = frame[1];
    chr_test[2] = frame[2];
    chr_test[3] = frame[3];

    mcSetDutyCycle(MC_CHANNEL_PWM1, duty_cycle[0]);
    //mcSetDutyCycle(1, duty_cycle[0]);
 */
}

static void cmdSteer(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
/*
    unsigned char chr_test[4];
    float *steer_value = (float*) chr_test;

    chr_test[0] = frame[0];
    chr_test[1] = frame[1];
    chr_test[2] = frame[2];
    chr_test[3] = frame[3];

    mcSteer(steer_value[0]);
    */
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

    legCtrlSetInput(LEG_CTRL_LEFT, argsPtr->chan1);
    legCtrlOnOff(LEG_CTRL_LEFT, PID_ON); //Motor PID #1 -> ON

    legCtrlSetInput(LEG_CTRL_RIGHT, argsPtr->chan2);
    legCtrlOnOff(LEG_CTRL_RIGHT, PID_ON); //Motor PID #2 -> ON
}

static void cmdSetPIDGains(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    //Unpack unsigned char* frame into structured values
    PKT_UNPACK(_args_cmdSetPIDGains, argsPtr, frame);

    legCtrlSetGains(0, argsPtr->Kp1, argsPtr->Ki1, argsPtr->Kd1, argsPtr->Kaw1, argsPtr->Kff1);
    legCtrlSetGains(1, argsPtr->Kp2, argsPtr->Ki2, argsPtr->Kd2, argsPtr->Kaw2, argsPtr->Kff2);

    //Send confirmation packet, which is the exact same data payload as what was sent
    //Note that the destination is the hard-coded RADIO_DST_ADDR
    //todo : extract the destination address properly.
    radioSendData(src_addr, 0, CMD_SET_PID_GAINS, length, frame, 0);

}

static void cmdGetPIDTelemetry(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    //Obsolete, not maintained
   
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

static void cmdGetImuLoopZGyro(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    //Obsolete, do not use
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
    _T4IE = 0;

    telemReadbackSamples(argsPtr->samples, src_addr);

    //Horibble hack: Disable IMU while erading back telem
    _T4IE = 1;
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
static void cmdSetVelProfile(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
/*
    //Unpack unsigned char* frame into structured values
    PKT_UNPACK(_args_cmdSetVelProfile, argsPtr, frame);

    hallSetVelProfile(0, argsPtr->intervalsL, argsPtr->deltaL, argsPtr->velL);
    hallSetVelProfile(1, argsPtr->intervalsR, argsPtr->deltaR, argsPtr->velR);

    //Send confirmation packet; this only happens when flash erase is completed.
    //Note that the destination is the hard-coded RADIO_DST_ADDR
    //todo : extract the destination address properly.
    radioSendData(RADIO_DST_ADDR, 0, CMD_SET_VEL_PROFILE, length, frame, 0);
 */
}

// report motor position and  reset motor position (from Hall effect sensors)
// note motor_count is long (4 bytes)
void cmdZeroPos(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
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

// alternative telemetry which runs at 1 kHz rate inside PID loop
static void cmdHallTelemetry(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    //TODO: Integration of hall telemetry is unfinished. Fuction will currently
    // do nothing.

    //This is only commented to supress the warning
    //PKT_UNPACK(_args_cmdHallTelemetry, argsPtr, frame);
     
}

// send robot info when queried
void cmdWhoAmI(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    // maximum string length to avoid packet size limit
    char* verstr = versionGetString();
    int verlen = strlen(verstr);

    //Note that the destination is the hard-coded RADIO_DST_ADDR
    //todo : extract the destination address properly.
    radioSendData(src_addr, 0, CMD_WHO_AM_I, verlen, (unsigned char*)verstr, 0);
}

static void cmdSetHallGains(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    /*
    
    //Unpack unsigned char* frame into structured values
    PKT_UNPACK(_args_cmdSetPIDGains, argsPtr, frame);

    hallSetGains(0, argsPtr->Kp1, argsPtr->Ki1, argsPtr->Kd1, argsPtr->Kaw1, argsPtr->Kff1);
    hallSetGains(1, argsPtr->Kp2, argsPtr->Ki2, argsPtr->Kd2, argsPtr->Kaw2, argsPtr->Kff2);

    //Note that the destination is the hard-coded RADIO_DST_ADDR
    //todo : extract the destination address properly.
    radioSendData(RADIO_DST_ADDR, 0, CMD_SET_HALL_GAINS, length, frame, 0);
     */
}

static void cmdSetTailQueue(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    //Tail control being deprecated from mainline project
    //TODO () : figure out an easy way of switching in tail control
    /*
    //The count is read via standard pointer math
    unsigned int count;
    count = (unsigned int) (*(frame));

    //Unpack unsigned char* frame into structured values
    //Here, unpacking happens in the loop below.
    //Due to variable size, PKT_UNPACK is not used in this CMD
    int idx = sizeof (count); //should be an unsigned int, sizeof(uint) = 2

    tailCmdT tailSeg;
    int i;
    for (i = 0; i < count; i++) {
        tailSeg = (tailCmdT) malloc(sizeof (tailCmdStruct));
        //argsPtr = (_args_cmdSetMoveQueue*)(frame+idx);
        *tailSeg = *((tailCmdT) (frame + idx));
        tailqPush(tailq, tailSeg);
        //idx =+ sizeof(_args_cmdSetMoveQueue);
        idx += sizeof (tailCmdStruct);
    }
  */
}

static void cmdSetTailGains(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    //Tail control being deprecated from mainline project
    //TODO () : figure out an easy way of switching in tail control
    /*
    //Unpack unsigned char* frame into structured values
    PKT_UNPACK(_args_cmdSetTailGains, argsPtr, frame);

    tailCtrlSetGains(argsPtr->Kp, argsPtr->Ki, argsPtr->Kd, argsPtr->Kaw, argsPtr->Kff);

    //Note that the destination is the hard-coded RADIO_DST_ADDR
    //todo : extract the destination address properly.
    radioSendData(RADIO_DST_ADDR, 0, CMD_SET_TAIL_GAINS, length, frame, 0);
     */
}


static void cmdSetThrustHall(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    /*
    //Unpack unsigned char* frame into structured values
    PKT_UNPACK(_args_cmdSetThrustHall, argsPtr, frame);

    hallPIDSetInput(0 , argsPtr->chan1, argsPtr->runtime1);
    hallPIDOn(0);
    hallPIDSetInput(1 , argsPtr->chan1, argsPtr->runtime2);
    hallPIDOn(1);
     */
}

static void cmdSetOLVibe(unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    //Unpack unsigned char* frame into structured values
    PKT_UNPACK(_args_cmdSetOLVibe, argsPtr, frame);

    olVibeSetFrequency(argsPtr->channel, argsPtr->incr);
    olVibeSetPhase(argsPtr->channel, argsPtr->phase);
    olVibeSetAmplitude(argsPtr->channel, argsPtr->amplitude);
}