"""
authors: apullin

Contents of this file are copyright Andrew Pullin, 2013

"""

import msvcrt, sys, time
from lib import command 
import shared

from or_helpers import *


###### Operation Flags ####
RESET_ROBOT = True   #Note: This MUST be False if you're using an XBee
                      # This is a known bug.


def menu():
    print "-------------------------------------"
    print "Keyboard control with Mixing PLUS Aux AMS+Hbridge"
    print " up: increase throt     q:quit   left/right: steering"
    print " down: decrease throt   s: reset to straight  space: all stop"
    print " t: AUX up 10.0 deg   g: AUX down 10.0 deg"
    print " z: AUX to 0 deg"
    print ""

def main():
    xb = setupSerial(shared.BS_COMPORT, shared.BS_BAUDRATE)
    shared.xb = xb
    
    R1 = Robot('\x20\x52', xb)
    #R1 = Robot('\x20\x27', xb)
    shared.ROBOTS = [R1]

    if RESET_ROBOT:
        print "Resetting robot..."
        R1.reset()
        time.sleep(0.5)

    motorgains = [15000,500,0,0,20,    15000,500,0,0,20]
    R1.setMotorGains(motorgains, retries = 8)
    
    verifyAllMotorGainsSet()  #exits on failure
    
    tailgains = [15000,100,100,0,0]
    R1.setTailGains(tailgains, retries = 8)
    verifyAllTailGainsSet()
    
    tinc = 30;

    forward = 0
    turn = 0
    
    left_throt = 0
    right_throt = 0
    
    auxangle = 0.0

    #blank out any keypresses leading in...
    while msvcrt.kbhit():
        ch = msvcrt.getch()
    menu()  #Display command menu
    while True:
        keypress = msvcrt.getch()
        if keypress == ' ':
            forward = 0
            turn = 0
        elif ord(keypress) == 72: #up
            forward = forward + tinc
        elif ord(keypress) == 80: #down
            forward = forward - tinc
        elif ord(keypress) == 75: #left
            turn = turn - tinc
        elif ord(keypress) == 77: #right
            turn = turn + tinc
        elif keypress == 't': #AUX up 15.0
            auxangle = auxangle + 10.0
            tailq = [1, auxangle, 500, TAIL_SEG_CONSTANT, 0, 0, 0]
            R1.sendTailQueue(tailq)
        elif keypress == 'g': #AUX down 15.0
            auxangle = auxangle - 10.0
            tailq = [1, auxangle, 500, TAIL_SEG_CONSTANT, 0, 0, 0]
            R1.sendTailQueue(tailq)
        elif keypress == 'z': #AUX to 0.0
            auxangle = 0.0
            tailq = [1, auxangle, 500, TAIL_SEG_CONSTANT, 0, 0, 0]
            R1.sendTailQueue(tailq)
        elif keypress == 's':  #go straight
                forward = min(left_throt, right_throt)
                turn = 0
        elif (keypress == 'q') or (ord(keypress) == 26):
                xb_safe_exit()
                sys.exit(0)

        if forward < 0:
            forward = 0
        
        # Mixing for turning
        left_throt = forward - turn 
        right_throt = forward + turn
        
        # Clipping to 0
        if left_throt < 0:
            left_throt = 0
        if right_throt < 0:
            right_throt = 0
        
        #Display output values
        outstring = "L: {0:03.1f}  |   R: {1:03.1f}      | AUX : {2:3.1f}      \r".format(left_throt,right_throt,auxangle)
        sys.stdout.write(outstring)
        sys.stdout.flush()
        
        R1.setMotorSpeeds(left_throt, right_throt)
        ### FOR REVERSED MOTORS:
        #setMotorSpeeds(right_throt, left_throt)

        time.sleep(0.05) #efective debounce, or rate limiting



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
    except serial.serialutil.SerialException:
        shared.xb.halt()
        shared.ser.close()
