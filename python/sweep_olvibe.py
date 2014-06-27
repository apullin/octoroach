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
    
    shared.ROBOTS = [R1] #This is neccesary so callbackfunc can reference robots
    shared.xb = xb           #This is neccesary so callbackfunc can halt before exit

    if RESET_R1:
        R1.reset()
        time.sleep(0.35)
    
    # Query
    R1.query( retries = 8 )
    
    #Verify all robots can be queried
    verifyAllQueried()  #exits on failure

    START_FREQ = 10
    END_FREQ = 120
    STEP_FREQ = 2
    freq_sweep = range(START_FREQ,END_FREQ,STEP_FREQ)
    segtime = 2
    
    amp = 1000
    
    ##### Manually set number of samples to save , TODO: make this a function in or_helpers
    R1.runtime = len(freq_sweep)*segtime
    R1.numSamples = int((1000*R1.runtime + (0.5 + 0.5)*1000))
    #allocate an array to write the downloaded telemetry data into
    R1.imudata = [ [] ] * R1.numSamples
    R1.moveq = ['OL Vibe sweep, ' + str(freq_sweep) + ' hz , amp = ' + str(amp)]
    R1.eraseFlashMem(timeout = 1000)
    
    
    # Pause and wait to start run, including leadin time
    print ""
    print "  ***************************"
    print "  *******    READY    *******"
    print "  ***************************"
    raw_input("  Press ENTER to start run ...")
    print "2 second settling time ... "
    time.sleep(2)
    
    if SAVE_DATA1:
        R1.startTelemetrySave()
    
    #Lead-in
    time.sleep(0.5)
    
    
    
    #freq_sweep = range(30,120,3)
    
    for freq in freq_sweep:
        R1.setOLVibe(1, freq, amp)
        R1.setOLVibe(2, freq, amp)
        print "Freq = ",freq
        time.sleep(segtime)
        
    R1.setOLVibe(1, freq, 0)
    R1.setOLVibe(2, freq, 0)
    
    #Lead-out
    time.sleep(0.5)
    
    #telemetry download
    raw_input("Press Enter to start telemtry readback ...")
    R1.downloadTelemetry()

        
    #End of script handler
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
