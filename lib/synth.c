#include "synth.h"

static void synthUpdate() {
    //Move segment synthesis
    long ySL = currentMove->inputL; //store in local variable to limit lookups
    long ySR = currentMove->inputR; // "
    int yL = 0;
    int yR = 0;
    if (inMotion) {
        if (currentMove->type == MOVE_SEG_IDLE) {
            yL = 0;
            yR = 0;
        }
        if (currentMove->type == MOVE_SEG_CONSTANT) {
            yL = ySL;
            yR = ySR;
        }
        if (currentMove->type == MOVE_SEG_RAMP) {
            long rateL = (long) currentMove->params[0];
            long rateR = (long) currentMove->params[1];
            //Do division last to prevent integer math underflow
            yL = rateL * ((long) t1_ticks - (long) currentMoveStart) / 1000 + ySL;
            yR = rateR * ((long) t1_ticks - (long) currentMoveStart) / 1000 + ySR;
        }
        if (currentMove->type == MOVE_SEG_SIN) {
            //float temp = 1.0/1000.0;
            float amp = (float) currentMove->params[0];
            //float F = (float)currentMove->params[1] / 1000;
            float F = (float) currentMove->params[1] * 0.001;
#define BAMS16_TO_FLOAT 1/10430.367658761737
            float phase = BAMS16_TO_FLOAT * (float) currentMove->params[2]; //binary angle
            //Must be very careful about underflow & overflow here!
            //long arg = 2*BAMS16_PI*mF/100;
            //arg = arg*(t1_ticks - currentMoveStart)/10 - phase;
            //float fyL = amp*sin(2*3.1415*F*(float)(t1_ticks - currentMoveStart)/1000  - phase) + ySL;
            //float fyR = amp*sin(2*3.1415*F*(float)(t1_ticks - currentMoveStart)/1000  - phase) + ySR;
            float fyL = amp * sin(2 * 3.1415 * F * (float) (t1_ticks - currentMoveStart)*0.001 - phase) + ySL;
            float fyR = amp * sin(2 * 3.1415 * F * (float) (t1_ticks - currentMoveStart)*0.001 - phase) + ySR;


            //fractional arg = 2*(long)F*((long)t1_ticks-(long)currentMoveStart) *
            //fractional temp = _Q15sinPI(arg);
            //fractional wave = (int)((long)temp*(long)amp >> 15);

            //Clipping
            int temp = (int) fyL;
            if (temp < 0) {
                temp = 0;
            }
            yL = (unsigned int) temp;
            temp = (int) fyR;
            if (temp < 0) {
                temp = 0;
            }
            yR = (unsigned int) temp;
            //unsigned int yL = amp*sin(arg) + ySL;
        }
        motor_pidObjs[0].input = yL;
        motor_pidObjs[1].input = yR;
    }
    //Note hhere that pidObjs[n].input is not set if !inMotion, in case another behavior wants to
    // set it.
}

