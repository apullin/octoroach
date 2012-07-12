/* Queue (FIFO) for moveCmdT

 */

#include "hall_queue.h"
#include "hall.h"
#include "p33Fxxxx.h"
#include <stdio.h>      // for NULL
#include <stdlib.h>     // for malloc


/*-----------------------------------------------------------------------------
 *          Public functions
-----------------------------------------------------------------------------*/

HallQueue hallqInit(int max_size) {
    Queue hq = queueInit(max_size);
    return hq;
}

void hallqPush(HallQueue hq, hallCmdT newItem) {

    hallCmdT temp;

    if (queueIsFull(hq)) {
        temp = (hallCmdT)queuePop(hq);
        free(temp);
    }

    queueAppend(hq, newItem);

}

hallCmdT hallqPop(HallQueue queue) {
    return (hallCmdT)queuePop(queue);
}

int tailqIsFull(HallQueue queue) {
    return queueIsFull(queue);
}

int tailqIsEmpty(HallQueue queue) {
    return queueIsEmpty(queue);
}


int tailqGetSize(HallQueue queue) {
    return queueGetSize(queue);
}

