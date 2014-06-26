// Contents of this file are copyright Andrew Pullin, 2013

//or_telem.h , OctoRoACH specific telemetry packet format header

#include <stdint.h>

// Data structure type
typedef struct {
    int16_t inputL;
    int16_t inputR;
    int16_t dcA;
    int16_t dcB;
    int16_t dcC;
    int16_t dcD;
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
    int16_t gyroAvg;
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
    int16_t bemfA;
    int16_t bemfB;
    int16_t bemfC;
    int16_t bemfD;
    int16_t steerIn;
    int16_t steerOut;
    int16_t Vbatt;
    float yawAngle;
} orTelemStruct_t;

void orTelemGetData(unsigned char* ptr);

unsigned int orTelemGetSize();