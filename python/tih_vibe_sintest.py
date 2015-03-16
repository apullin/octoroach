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
RESET_R1 = True  
SAVE_DATA1 = True
EXIT_WAIT   = False

def main():    
    xb = setupSerial(shared.BS_COMPORT, shared.BS_BAUDRATE)
    
    R1 = Robot('\x20\x52', xb)
    R1.SAVE_DATA = True
    
    shared.ROBOTS = [R1] #This is neccesary so callbackfunc can reference robots
    shared.xb = xb           #This is neccesary so callbackfunc can halt before exit    
    
    if RESET_R1:
        R1.reset()
        time.sleep(0.35)
    
    # Query
    R1.query( retries = 8 )
    
    #Verify all robots can be queried
    verifyAllQueried()  #exits on failure

    ##### Manually set number of samples to save , TODO: make this a function in or_helpers
    R1.runtime = 1;
    R1.numSamples = int((1000*R1.runtime + (0.25 + 0.25)*1000))
    #allocate an array to write the downloaded telemetry data into
    R1.imudata = [ [] ] * R1.numSamples
    R1.moveq = ['none']
    if SAVE_DATA1:
        R1.eraseFlashMem(timeout = 1000)
    
    raw_input("Press enter to start vibe...")
    if SAVE_DATA1:
        R1.startTelemetrySave()
        
    #Lead-in
    time.sleep(0.25)
    
    freqL = 10.0
    freqR = 10.0
    amp = 3900
    phase = 0.0
    R1.setOLVibe(1, freqL, amp, phase)
    R1.setOLVibe(2, freqR, amp, phase)
    time.sleep(1)
    R1.setOLVibe(1, freqL, 0, phase)
    R1.setOLVibe(2, freqR, 0, phase)
    
    #Lead-out
    time.sleep(0.25)
    
    # Initiate telemetry recording; the robot will begin recording immediately when cmd is received.
    for r in shared.ROBOTS:
        if r.SAVE_DATA:
            r.startTelemetrySave()

    if EXIT_WAIT:  #Pause for a Ctrl + Cif specified
        while True:
            try:
                time.sleep(1)
            except KeyboardInterrupt:
                break

    print "Done"
    xb_safe_exit()

	
	
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
    except Exception as args:
        print "\nGeneral exception:",args
        print "Attemping to exit cleanly..."
        shared.xb.halt()
        shared.ser.close()
        sys.exit()
