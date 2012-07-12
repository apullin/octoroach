#ifndef __HALL_QUEUE_H
#define __HALL_QUEUE_H

#include "queue.h"
#include "hall.h"
//#include "pid.h"

enum hallSegT{
	HALL_SEG_CONSTANT,
	HALL_SEG_IDLE,
        HALL_SEG_LOOP_DECL,
        HALL_SEG_LOOP_CLEAR,
        HALL_SEG_QFLUSH
};

typedef struct
{
    hallVelLUT table;
    unsigned long duration;
    enum hallSegT type;
} hallCmdStruct;

typedef hallCmdStruct* hallCmdT;

//typedef generic pointer type, Item;
typedef Queue HallQueue;

HallQueue hallqInit(int max_size);

void hallqPush(HallQueue hq, hallCmdT new_item);

hallCmdT hallqPop(HallQueue queue);

int hallqIsFull(HallQueue queue);

int hallqIsEmpty(HallQueue queue);

int hallqGetSize(HallQueue queue);

#endif // __HALL_QUEUE_H
