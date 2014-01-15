#!/usr/bin/env python
"""
cmd module
authors: stanbaek, apullin

Modifications and additions to this file made by Andrew Pullin are copyright, 2013
Copyrights are acknowledged for portions of this code extant before modifications by Andrew Pullin 
Any application of BSD or other license to copyright content without the authors express approval
is invalid and void.

"""

class commandEntry:    
    def __init__(self, num, name, func):
        self.cmdNum = num
        self.cmdName = name
        self.cmdFunction = func
        

# CMD values of 0x00 - 0x7F(127) are defined here
# Add CMD definitions 
# for bootloader (0x00 - 0x1F)
CMD_NACK = 0x00        # START_APPLICATION = 0
CMD_ACK = 0x01
CMD_READ_PM = 0x02
CMD_WRITE_PM = 0x03
CMD_READ_EE = 0x04
CMD_WRITE_EE = 0x05
CMD_READ_CM = 0x06
CMD_WRITE_CM = 0x07
CMD_RESET = 0x08
CMD_READ_ID = 0x09
CMD_READ_GOTO = 0x10

SET_THRUST = 0x11
SET_STEER = 0x12
ECHO = 0x1F      # send back the received packet

# for IMU (0x20 - 0x3F)
GET_IMU_DATA = 0x20
GET_IMU_LOOP = 0x21
START_IMU_SAVE = 0x22
STOP_IMU_SAVE = 0x23

SET_POSE_SAVE_FLASH = 0x25
SET_ESTIMATE_POSE = 0x26

TX_SAVED_IMU_DATA = 0x2A
TX_SAVED_STATE_DATA = 0x2B
TX_DUTY_CYCLE = 0x2C

START_AUTO_CTRL = 0x30
STOP_AUTO_CTRL = 0x31

ERASE_MEM_SECTOR = 0x3A
 
RESET_STOPWATCH = 0x3B

BASE_ECHO = 0x3f



                
cmdList = []

# CMD values of 0x80(128) - 0xEF(239) are available for user applications.
SET_THRUST_OPEN_LOOP =      0x80
cmdList.append( commandEntry(SET_THRUST_OPEN_LOOP, 'SET_THRUST_OPEN_LOOP', 'cmdSetThrustOpenLoop') )

SET_THRUST_CLOSED_LOOP =    0x81
cmdList.append( commandEntry(SET_THRUST_CLOSED_LOOP, 'SET_THRUST_CLOSED_LOOP', 'cmdSetThrustClosedLoop') )

SET_PID_GAINS =             0x82
cmdList.append( commandEntry(SET_PID_GAINS, 'SET_PID_GAINS', 'cmdSetPIDgains') )

GET_PID_TELEMETRY =         0x83
cmdList.append( commandEntry(GET_PID_TELEMETRY, 'GET_PID_TELEMETRY', 'cmdPIDtelemetry') )

SET_CTRLD_TURN_RATE =       0x84
cmdList.append( commandEntry(SET_CTRLD_TURN_RATE, 'SET_CTRLD_TURN_RATE' ,'cmdCtrldTurnRate') )

GET_IMU_LOOP_ZGYRO =        0x85
cmdList.append( commandEntry(GET_IMU_LOOP_ZGYRO, 'GET_IMU_LOOP_ZGYRO' , 'cmdIMUloopZgyro') )

SET_MOVE_QUEUE =            0x86
cmdList.append( commandEntry(SET_MOVE_QUEUE, 'SET_MOVE_QUEUE' , 'cmdSetMoveQueue') )

SET_STEERING_GAINS =        0x87
cmdList.append( commandEntry(SET_STEERING_GAINS, 'SET_STEERING_GAINS' , 'cmdSetSteeringGains') )

SOFTWARE_RESET     =        0x88
cmdList.append( commandEntry(SOFTWARE_RESET, 'SOFTWARE_RESET' , 'cmdSoftwareReset') )

SPECIAL_TELEMETRY  =        0x89
cmdList.append( commandEntry(SPECIAL_TELEMETRY, 'SPECIAL_TELEMETRY' , 'cmdSpecialTelemetry') )

ERASE_SECTORS      =        0x8A
cmdList.append( commandEntry(ERASE_SECTORS, 'ERASE_SECTORS' , 'cmdEraseSectors') )

FLASH_READBACK     =        0x8B
cmdList.append( commandEntry(FLASH_READBACK, 'FLASH_READBACK' , 'cmdFlashReadback') )

SLEEP              =        0x8C
cmdList.append( commandEntry(SLEEP, 'SLEEP' , 'cmdSleep') )

SET_VEL_PROFILE =           0x8D
cmdList.append( commandEntry(SET_VEL_PROFILE, 'SET_VEL_PROFILE' , 'cmdSetVelProfile') )

WHO_AM_I =                  0x8E
cmdList.append( commandEntry(WHO_AM_I, 'WHO_AM_I' ,'cmdWhoAmI') )

START_TELEM =               0x8F
cmdList.append( commandEntry(START_TELEM, 'START_TELEM' ,'cmdStartTelem') )

ZERO_POS =                  0x90
cmdList.append( commandEntry(ZERO_POS, 'ZERO_POS' ,'cmdZeroPos') )

SET_HALL_GAINS =            0x91
cmdList.append( commandEntry(SET_HALL_GAINS, 'SET_HALL_GAINS' ,'cmdSetHallGains') )

SET_TAIL_QUEUE =            0x92
cmdList.append( commandEntry(SET_TAIL_QUEUE, 'SET_TAIL_QUEUE' ,'cmdSetTailQueue') )

SET_TAIL_GAINS =            0x93
cmdList.append( commandEntry(SET_TAIL_GAINS, 'SET_TAIL_GAINS' ,'cmdSetTailGains') )

# CMD values of 0xF0(240) - 0xFF(255) are reserved for future use
