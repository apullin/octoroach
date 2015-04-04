#!/usr/bin/env python
"""
authors: apullin

Contents of this file are copyright Andrew Pullin, 2013

"""
from lib import command
import time,sys
import serial
import shared

from or_helpers import *


###### Operation Flags ####
SAVE_DATA1 = False 
RESET_R1 = True

EXIT_WAIT   = False

def main():    
    xb = setupSerial(shared.BS_COMPORT, shared.BS_BAUDRATE, rtscts=1)
    
    R1 = Robot('\x20\x52', xb)
    
    shared.ROBOTS = [R1] #This is neccesary so callbackfunc can reference robots
    shared.xb = xb           #This is neccesary so callbackfunc can halt before exit
    
    #if SAVE_DATA1:
    #    shared.dataFileName = findFileName();
    #    print "Data file:  ", shared.dataFileName

    if RESET_R1:
        R1.reset()
        time.sleep(0.35)
    
    # Query
    R1.query( retries = 8 )
    
    #Verify all robots can be queried
    verifyAllQueried()  #exits on failure

    
    #Motor gains format:
    #  [ Kp , Ki , Kd , Kaw , Kff     ,  Kp , Ki , Kd , Kaw , Kff ]
    #    ----------LEFT----------        ---------_RIGHT----------
    
    #motorgains = [15000,50,1000,0,0,    15000,50,1000,0,0] #Hardware PID
    motorgains = [20000,3000,50,0,0,    20000,3000,50,0,0]

    R1.setMotorGains(motorgains, retries = 4)
    #Verify all robots have motor gains set
    verifyAllMotorGainsSet()   #exits on failure

    #Steering gains format:
    #  [ Kp , Ki , Kd , Kaw , Kff]
    steeringGains = [0,0,0,0,0,  STEER_MODE_OFF] # Hardware PID

    R1.setSteeringGains(steeringGains, retries = 4)
    #Verify all robots have steering gains set
    verifyAllSteeringGainsSet()  #exits on failure
    
    #No movements, just for static telemetry capture
    numMoves = 1
    moveq1 = [numMoves, \
        0, 0, 2400,   MOVE_SEG_CONSTANT, 0,  0,  0, STEER_MODE_OFF, 0]    
    #### This move queue will not be used, it is only to calculate the number of samples we'll need! ####

    #Timing settings
    R1.leadinTime = 0;
    R1.leadoutTime = 0;
    
    #Flash must be erased to save new data
    if SAVE_DATA1:
        #This needs to be done to prepare the .imudata variables in each robot object
        R1.setupImudata(moveq1)
        R1.eraseFlashMem()

    # Pause and wait to start run, including leadin time
    print ""
    print "  ***************************"
    print "  *******    READY    *******"
    print "  ***************************"
    raw_input("  Press ENTER to start run ...")
    print ""
    
    # Trigger telemetry save, which starts as soon as it is received
    
    #### Make when saving anything, this if is set ####
    #### to the proper "SAVE_DATA"                 ####
    
    if SAVE_DATA1:
        R1.startTelemetrySave()
        
    # lead in
    time.sleep(8)
    
    #### OL Vibe seciton ####
    freq = 40
    amp = 2500
    runtime_sec = 3
    R1.setOLVibe(1, freq, amp)
    R1.setOLVibe(2, freq, amp)
    time.sleep(runtime_sec)
    R1.setOLVibe(1, freq, 0)
    R1.setOLVibe(2, freq, 0)
    
    #lead out
    time.sleep(0.2)
    
    R1.moveq = moveq1
    
    if SAVE_DATA1:
        raw_input("Press Enter to start telemtry readback ...")
        R1.downloadTelemetry()

    if EXIT_WAIT:  #Pause for a Ctrl + Cif specified
        while True:
            try:
                time.sleep(1)
            except KeyboardInterrupt:
                break

    print "Done"
    xb_safe_exit()


def trapRun(topspeed = 0, tstime = 0, acceltime = 0, deceltime = 0, steertype = STEER_MODE_YAW_DEC):
    moveq = []
    numMoves = 0
    if acceltime != 0:
        ramprate = int(topspeed / ( acceltime/1000.0))
        moveq.extend( [ 0, 0, acceltime,   MOVE_SEG_RAMP, ramprate, ramprate,  0, steertype, 0])
        numMoves = numMoves + 1
        
    if tstime != 0:
        ramprate = int(topspeed / ( acceltime/1000.0))
        moveq.extend( [ topspeed, topspeed, tstime,   MOVE_SEG_CONSTANT, 0, 0,  0, steertype, 0])
        numMoves = numMoves + 1
        
    if deceltime != 0:
        ramprate = -int(topspeed / ( deceltime/1000.0))
        moveq.extend( [ topspeed, topspeed, deceltime,   MOVE_SEG_RAMP, ramprate, ramprate,  0, steertype, 0])
        numMoves = numMoves + 1
        
    moveq.insert(0,numMoves)
    
    return [numMoves, moveq]


#Provide a try-except over the whole main function
# for clean exit. The Xbee module should have better
# provisions for handling a clean exit, but it doesn't.
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print "\nRecieved Ctrl+C, exiting."
        shared.xb.halt()
        shared.ser.close()
    #except Exception as args:
    #    print "\nGeneral exception:",args
    #    print "Attemping to exit cleanly..."
    #    shared.xb.halt()
    #    shared.ser.close()
    #    sys.exit()
