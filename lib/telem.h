// Contents of this file are copyright Andrew Pullin, 2013

#ifndef __TELEM_H
#define __TELEM_H

//Telemetry packet structure
//This is specific to apullin's OctoROACH code

typedef struct {
    unsigned long sampleIndex;
    unsigned long timeStamp;
    int inputL;
    int inputR;
    int dcL;
    int dcR;
    int gyroX;
    int gyroY;
    int gyroZ;
    int gyroAvg;
    int accelX;
    int accelY;
    int accelZ;
    int bemfL;
    int bemfR;
    int Vbatt;
    int steerInput;
    float bodyPosition;
    unsigned long motor_count[2];
    int steerOutput;
} telemStruct_t;

//static char TELEM_UNPACK_FORMAT[] = "=LLhhhhhhhhhhhhhhhfLLh";
//static char TELEM_SAVE_FORMAT[]   = "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%d,%d,%d";

//TODO: A union is not neccesary here. Remove for clarity, and chage related
//  code

typedef union packedTelemUnion {
    telemStruct_t telemStruct;
    unsigned char dataArray[sizeof (telemStruct_t)];
} telemU;

#define PACKETSIZE sizeof(telemStruct_t)

#define TELEM_STREAM_OFF  0
#define TELEM_STREAM_ON   1

// Prototypes
void telemSetup(); //To be called in main
void telemReadbackSamples(unsigned long);
void telemSendDataDelay(unsigned char, unsigned char*, int delaytime_ms);
void telemSaveData(telemU *data);
void telemSetSamplesToSave(unsigned long n);
void telemErase(unsigned long);
void telemSetSkip(unsigned int skipnum);
void telemSetStartTime(void);

#endif  // __TELEM_H
