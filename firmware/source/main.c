/******************************************************************************
 * Name: main.c
 * Desc:
 * Date: 2010-07-08
 * Author: stanbaek, apullin
 Modifications and additions to this file made by Andrew Pullin are copyright, 2013
 Copyrights are acknowledged for portions of this code extant before modifications by Andrew Pullin 
 Any application of BSD or other license to copyright content without the authors express approval
 is invalid and void.
 *******************************************************************************/

#include <xc.h>

#include "settings.h"
#include "Generic.h"

#include "init_default.h"
#include "ports.h"
#include "battery.h"
#include "cmd.h"
#include "radio.h"
#include "mpu6000.h"
#include "utils.h"
#include "sclock.h"
#include "dfmem.h"
#include "leg_ctrl.h"
#include "pid.h"
#include "adc_pid.h"
#include "steering.h"
#include "telem.h"
#include "hall.h"
//#include "tail_ctrl.h"
//#include "ams-enc.h"
#include "tih.h"
#include "imu.h"
#include "spi_controller.h"
#include "ppool.h"

#include <stdlib.h>

extern unsigned char id[4];

int main(void) {

    //wakeTime = 0;
    //dcCounter = 0;

    // Processor Initialization
    SetupClock();
    SwitchClocks();
    SetupPorts();
    sclockSetup();

    LED_1 = 0;
    LED_2 = 0;
    LED_3 = 0;

    cmdSetup();
    
    radioInit(RADIO_TXPQ_MAX_SIZE, RADIO_RXPQ_MAX_SIZE);
    radioSetChannel(RADIO_CHANNEL);
    radioSetSrcPanID(RADIO_PAN_ID);
    radioSetSrcAddr(RADIO_SRC_ADDR);

    dfmemSetup();
    telemSetup(); //Timer 5, HW priority 4

    mpuSetup();
    imuSetup();   //Timer 4, HW priority 3
    
    tiHSetup();
    adcSetup();

    //AMS Encoders
    //encSetup();

    //"Open Loop" vibration & jitter generator, AP 2014
    //olVibeSetup();

    legCtrlSetup();  //Timer 1, HW priority 5
    steeringSetup(); //Timer 5, HW priority 4

    //Tail control is a special case
    //tailCtrlSetup();

    //Camera is untested with current code base, AP 12/6/2012
    //ovcamSetup();

    LED_RED = 1; //Red is use an "alive" indicator
    LED_GREEN = 0;
    LED_YELLOW = 0;

    //Radio startup verification
    //if (phyGetState() == 0x16) {
    //    LED_GREEN = 1;
    //}

    //Sleeping and low power options
    //_VREGS = 1;
    //gyroSleep();

    /////FUNCTION TEST, NOT FOR PRODUCTION
    //olVibeStart();
    ////////////////////

    while (1) {
        cmdHandleRadioRxBuffer();
        radioProcess();
    }
}
