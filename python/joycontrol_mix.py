"""
authors: apullin

Contents of this file are copyright Andrew Pullin, 2013

"""

from lib import command
import time,sys
import serial
import shared
import pygame

from or_helpers import *

###### Operation Flags ####
SAVE_DATA   = False
RESET_ROBOT = True
EXIT_WAIT   = False

MAXTHROT = 100

BUTTON_Y = 3
BUTTON_B = 1
BUTTON_L1 = 4
BUTTON_R1 = 5
BUTTON_X = 2
BUTTON_A = 0


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
            turnInput = -j.get_axis(4)
            
            if j.get_button(BUTTON_B) == 1 and MAXTHROT > 0:
                MAXTHROT = MAXTHROT - tinc
            elif j.get_button(BUTTON_Y) ==1 and MAXTHROT < 416:
                MAXTHROT = MAXTHROT + tinc
            
            
            
            DEADBAND = 0.05
            if abs(turnInput) < DEADBAND:
                turnInput = 0
            if (abs(thrustInput) < DEADBAND):
                thrustInput = 0
            
            left_throt = int(-turnInput * MAXTHROT + MAXTHROT*thrustInput )
            right_throt = int(turnInput * MAXTHROT + MAXTHROT*thrustInput )
            
            if left_throt > MAXTHROT:
                left_throt = MAXTHROT
            if left_throt < -MAXTHROT:
                left_throt = -MAXTHROT
            if right_throt > MAXTHROT:
                right_throt = MAXTHROT
            if right_throt < -MAXTHROT:
                right_throt = -MAXTHROT
            
            #### OL Vibe seciton ####
            amp = 3000
            if j.get_button(BUTTON_X) == 1:
                olVibeFreq = olVibeFreq + vinc
            if j.get_button(BUTTON_A) == 1:
                olVibeFreq = olVibeFreq - vinc
            if olVibeFreq < 0:
                olVibeFreq = 0;
            
            
            #if  (j.get_button(BUTTON_R1) == 1) and (LAST_BUTTON_R1 == 0):
            if  (j.get_button(BUTTON_R1) == 1):
                VIBRATION_ON = 1
                R1.setOLVibe(1, olVibeFreq, amp)
                R1.setOLVibe(2, olVibeFreq, amp)
            #if (j.get_button(BUTTON_R1) == 0) and (LAST_BUTTON_R1 == 1):
            if (j.get_button(BUTTON_R1) == 0):
                VIBRATION_ON = 0
                R1.setOLVibe(1, olVibeFreq, 0)
                R1.setOLVibe(2, olVibeFreq, 0)
                
            #LAST_BUTTON_R1 = j.get_button(BUTTON_R1)
            
            sys.stdout.write(" "*60 + "\r")
            sys.stdout.flush()
            outstring = "L: {0:3d}  |   R: {1:3d}   MAX: {2:1d}    Vibe: {3:0.1f}".format(left_throt,right_throt,MAXTHROT,olVibeFreq)
            #outstring = outstring + "(" + str(MAXTHROT) + ")\r"
            if VIBRATION_ON == 1:
                outstring = outstring + "    ((!!))"
            outstring = outstring + "\r"
            sys.stdout.write(outstring)
            sys.stdout.flush()
            
            throt = [left_throt,right_throt]
            if throt != lastthrot and (VIBRATION_ON == 0): #Only send new packet if throttles have changed
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
