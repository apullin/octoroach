// Contents of this file are copyright Andrew Pullin, 2013

//or_telem.h , OctoRoACH specific telemetry packet format header

// Data structure type
typedef struct {
    int inputL;
    int inputR;
    int dcA;
    int dcB;
    int dcC;
    int dcD;
    int gyroX;
    int gyroY;
    int gyroZ;
    int gyroAvg;
    int accelX;
    int accelY;
    int accelZ;
    int bemfA;
    int bemfB;
    int bemfC;
    int bemfD;
    int steerIn;
    int steerOut;
    int Vbatt;
    float yawAngle;
} orTelemStruct_t;

void orTelemGetData(unsigned char* ptr);

unsigned int orTelemGetSize();