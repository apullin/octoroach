"""
authors: apullin

Contents of this file are copyright Andrew Pullin, 2013

"""

from lib import command
import time,sys,os
import serial
import pygame

# Path to imageproc-settings repo must be added
sys.path.append(os.path.dirname("../../imageproc-settings/"))
sys.path.append(os.path.dirname("../imageproc-settings/"))  
import shared_multi as shared

from or_helpers import *

###### Operation Flags ####
SAVE_DATA   = False
RESET_ROBOT = True
EXIT_WAIT   = False

MAXTHROT = 100

#For PS3 controller
BUTTON_X = 0
BUTTON_CIRC = 1
BUTTON_SQUARE = 2
BUTTON_TRI = 3
# up, down, left, right are in a "hat"
# L1, R1 are axes
BUTTON_L2 = 4
BUTTON_L3 =  8
BUTTON_R2 = 5
BUTTON_R3 =   9


def main():
    global MAXTHROT
    xb = setupSerial(shared.BS_COMPORT, shared.BS_BAUDRATE)
    shared.xb = xb
    
    R1 = Robot('\x20\x52', xb)

    shared.ROBOTS = [R1]

    if RESET_ROBOT:
        print "Resetting robot..."
        R1.reset()
        time.sleep(0.5)
        
    # Send robot a WHO_AM_I command, verify communications
    R1.query(retries = 1)

    motorgains = [22000,0,500,0,0,    22000,0,500,0,0]
    #motorgains = [25000,50,0,0,25,    25000,50,0,0,25]

    R1.setMotorGains(motorgains, retries = 1)
    
    #verifyAllMotorGainsSet()  #exits on failure
    
    steeringgains = [0, 0, 0, 0, 0, 0]
    #R1.setSteeringGains(steeringgains)
    
    #verifyAllSteeringGainsSet()  #exits on failure
        
    j = setupJoystick()
    
    lastthrot = [0, 0]
    
    tinc = 25;
    olVibeFreq = 20;
    vinc = 2.5;
    LAST_BUTTON_R1 = 0;
    VIBRATION_ON = 0;

    while True:

            value = []
            pygame.event.pump()
            
            thrustInput = -j.get_axis(1)
            turnInput = j.get_axis(4)
            
            hatval = j.get_hat(0)
            
            if hatval[1] == 1:
                MAXTHROT = MAXTHROT + tinc
            elif hatval[1] == -1:
                MAXTHROT = MAXTHROT - tinc
            
            if MAXTHROT < 0:
                MAXTHROT = 0
            if MAXTHROT > 450:
                MAXTHROT = 450
            
            DEADBAND = 0.05
            if abs(turnInput) < DEADBAND:
                turnInput = 0
            if (abs(thrustInput) < DEADBAND):
                thrustInput = 0
            
            left_throt = int(turnInput * MAXTHROT + MAXTHROT*thrustInput )
            right_throt = int(-turnInput * MAXTHROT + MAXTHROT*thrustInput )
            
            # positive and negative clipping
            if left_throt > MAXTHROT:
                left_throt = MAXTHROT
            if left_throt < -MAXTHROT:
                left_throt = -MAXTHROT
            if right_throt > MAXTHROT:
                right_throt = MAXTHROT
            if right_throt < -MAXTHROT:
                right_throt = -MAXTHROT
            
            sys.stdout.write(" "*60 + "\r")
            sys.stdout.flush()
            outstring = "L: {0:3d}  |   R: {1:3d}   MAX: {2:1d} ".format(left_throt,right_throt,MAXTHROT)
            #outstring = outstring + "(" + str(MAXTHROT) + ")\r"
            outstring = outstring + "\r"
            sys.stdout.write(outstring)
            sys.stdout.flush()
            
            throt = [left_throt,right_throt]
            if throt != lastthrot: #Only send new packet if throttles have changed
                R1.setMotorSpeeds(left_throt, right_throt)
                lastthrot = throt
                
            time.sleep(.1)


def setupJoystick():
    try:
        pygame.init()
        j = pygame.joystick.Joystick(0)
        j.init()
        print j.get_name()
    except Exception as args:
        print 'No joystick'
        print 'Exception: ', args
        xb_safe_exit()
        
    return j

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
