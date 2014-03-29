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

    motorgains = [20000,100,0,0,10 , 20000,100,0,0,10]
    #motorgains = [25000,50,0,0,25,    25000,50,0,0,25]

    R1.setMotorGains(motorgains, retries = 8)
    
    verifyAllMotorGainsSet()  #exits on failure
    
    steeringgains = [0, 0, 0, 0, 0, 0]
    R1.setSteeringGains(steeringgains)
    
    verifyAllSteeringGainsSet()  #exits on failure
        
    # Send robot a WHO_AM_I command, verify communications
    R1.query()

    j = setupJoystick()
    
    lastthrot = [0, 0]
    
    tinc = 25;

    while True:

            value = []
            pygame.event.pump()
            
            thrustInput = -j.get_axis(1)
            turnInput = -j.get_axis(4)
            
            if j.get_button(BUTTON_B) == 1 and MAXTHROT > 0:
                MAXTHROT = MAXTHROT - tinc
            elif j.get_button(BUTTON_Y) ==1 and MAXTHROT < 416:
                MAXTHROT = MAXTHROT + tinc
            
            #TAIL FLICK
            ## Hard:
            tailOutPWM = 4000
            tailOutTime = 0.07
            tailStopPWM = -1500
            tailStopTime = 0.04
            ## Soft:
            #tailOutPWM = 1000
            #tailOutTime = 0.07
            #tailStopPWM = -300
            #tailStopTime = 0.04
            if j.get_button(BUTTON_L1) == 1:
                #Tail flick right
                print "TAIL FLICK RIGHT"
                R1.setTIH(3,tailOutPWM)
                time.sleep(tailOutTime)
                R1.setTIH(3,tailStopPWM)
                time.sleep(tailStopTime)
                R1.setTIH(3,0)
            if j.get_button(BUTTON_R1) == 1:
                print "TAIL FLICK LEFT"
                #Tail flick left
                R1.setTIH(3,-tailOutPWM)
                time.sleep(tailOutTime)
                R1.setTIH(3,-tailStopPWM)
                time.sleep(tailStopTime)
                R1.setTIH(3,0)
            
            DEADBAND = 0.05
            if abs(turnInput) < DEADBAND:
                turnInput = 0
            if (abs(thrustInput) < DEADBAND):
                thrustInput = 0
            
            left_throt = int(-turnInput * MAXTHROT + MAXTHROT*thrustInput )
            right_throt = int(turnInput * MAXTHROT + MAXTHROT*thrustInput )
            #left_throt = int((1-turnInput) * MAXTHROT * thrustInput )
            #right_throt = int((1+turnInput) * MAXTHROT * thrustInput )
            
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
            outstring = "L: {0:3d}  |   R: {1:3d}  ".format(left_throt,right_throt)
            outstring = outstring + "(" + str(MAXTHROT) + ")\r"
            sys.stdout.write(outstring)
            sys.stdout.flush()
            
            throt = [left_throt,right_throt]
            if throt != lastthrot: #Only send new packet if throttles have changed
                #R1.setMotorSpeeds(left_throt, right_throt)
                lastthrot = throt
                
            time.sleep(0.1)


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
