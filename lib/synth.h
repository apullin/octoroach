#ifndef __SYNTH_H
#define __SYNTH_H

enum synthSegT {
    SYNTH_SEG_CONSTANT,
    SYNTH_SEG_RAMP,
    SYNTH_SEG_SIN,
    SYNTH_SEG_TRI,
    SYNTH_SEG_SAW,
    SYNTH_SEG_IDLE
};

typedef struct {
    int input;
    unsigned long startTime;
    unsigned long duration;
    enum synthSegT type;
    int params[3];
} synthObj;

void synthUpdate(synthObj* , unsigned long);

#endif