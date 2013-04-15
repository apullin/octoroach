#!/usr/bin/env python
"""
authors: stanbaek, apullin

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
    xb = setupSerial(shared.BS_COMPORT, shared.BS_BAUDRATE)
    
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

    
    tailGains = [15000,100,0,0,0]
    #tailGains = [8000,0,0,0,0] # try just P for rotary control
    R1.setTailGains(tailGains)
    #Verify all robots have tail gains set
    verifyAllTailGainsSet()  #exits on failure
   
    tailq = [1, 0.0, 1234, TAIL_SEG_CONSTANT, 0, 0, 0]
        
    #Timing settings
    R1.leadinTime = 0;
    R1.leadoutTime = 0;

    # Pause and wait to start run, including leadin time
    print ""
    print "  ***************************"
    print "  *******    READY    *******"
    print "  ***************************"
    raw_input("  Press ENTER to start run ...")
    print ""
    
    R1.sendTailQueue(tailq)

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
    #except Exception as args:
    #    print "\nGeneral exception:",args
    #    print "Attemping to exit cleanly..."
    #    shared.xb.halt()
    #    shared.ser.close()
    #    sys.exit()