#!/usr/bin/env python
"""
authors: apullin

Contents of this file are copyright Andrew Pullin, 2013

"""
from lib import command
import time,sys,os
import serial

# Path to imageproc-settings repo must be added
sys.path.append(os.path.dirname("../../imageproc-settings/"))
sys.path.append(os.path.dirname("../imageproc-settings/"))  
import shared_multi as shared

from or_helpers import *


###### Operation Flags ####
RESET_R1 = True  
SAVE_DATA1 = False  # CURRENTLY BROKEN HERE
EXIT_WAIT   = False

def main():    
    xb = setupSerial(shared.BS_COMPORT, shared.BS_BAUDRATE)
    
    R1 = Robot('\x20\x52', xb)
    
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
    R1.runtime = 4000; #milliseconds
    
    if SAVE_DATA1:
        R1.numSamples = int((R1.runtime + (0.25 + 0.25)*1000))
        print "Num samples:",R1.numSamples
        #allocate an array to write the downloaded telemetry data into
        R1.imudata = [ [] ] * R1.numSamples
        R1.moveq = ['none']
        R1.eraseFlashMem(timeout = 1000)
    
    raw_input("Press enter to start vibe...")
    
    time.sleep(0.25) #leadin
    
    ZERO_PHASE = 0
    #R1.setTIH(3,3999)
    freq = 30
    amp = 3500
    R1.setOLVibe(1, freq, amp, ZERO_PHASE)
    R1.setOLVibe(2, freq, amp, ZERO_PHASE)
    time.sleep( R1.runtime/1000.0 )
    R1.setOLVibe(1, freq, 0, ZERO_PHASE)
    R1.setOLVibe(2, freq, 0, ZERO_PHASE)
    #R1.setTIH(3,0)
    
    time.sleep(0.25) #leadout
    
    if SAVE_DATA1:
        #telemetry download
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
        print "Attempting to exit cleanly..."
        shared.xb.halt()
        shared.ser.close()
        sys.exit()
