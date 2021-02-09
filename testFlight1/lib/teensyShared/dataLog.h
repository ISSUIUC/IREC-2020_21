#ifndef DATALOG_H
#define DATALOG_H

#include <stdint.h>
#include <SD.h>

#include "acShared.h"

struct dataStruct_t {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float mx;
    float my;
    float mz;

    //! data for highGimu accel data (hg_x, hg_y, hg_z) 
    float hg_ax;
    float hg_ay;
    float hg_az;
    // int16_t pt1;
    // int16_t pt2;
    // int16_t pt3;

    //GPS DATA
    float latitude;
    float longitude;
    float altitude;
    bool posLock;
    
    int32_t timeStamp;

};

void init_dataLog(File* dataFile);

void logData(File* dataFile, dataStruct_t* data, FSM_State rocketState);

#endif
