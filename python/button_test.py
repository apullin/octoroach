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
    
    j = setupJoystick()
    

    while True:

            value = []
            pygame.event.pump()
            
            for i in range(j.get_numaxes()):
                print "axis",i," : ", j.get_axis(i)
            
            for i in range(j.get_numbuttons()):
                print "button",i," : ", j.get_button(i)
            
            for i in range(j.get_numhats()):
                print "hat",i," : ", j.get_hat(i)
            
                
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
        sys.exit()
