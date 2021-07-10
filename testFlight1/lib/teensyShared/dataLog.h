#ifndef DATALOG_H
#define DATALOG_H

#include <stdint.h>
#include <SD.h>
#include <ChRt.h>

#include "acShared.h"

#include "dataStructs.h"


struct lowGData {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float mx;
    float my;
    float mz;
    int32_t timeStamp_lowG;
};

struct highGData {
    float hg_ax;
    float hg_ay;
    float hg_az;
    int32_t timeStamp_highG;
};

struct gpsData {
    float latitude;
    float longitude;
    float altitude;
    bool posLock;
    int32_t timeStamp_GPS;
};

struct stateData {
    float state_q0 = 0;
    float state_q1 = 0;
    float state_q2 = 0;
    float state_q3 = 0;

    float state_x = 0;
    float state_y = 0;
    float state_z = 0;

    float state_vx = 0;
    float state_vy = 0;
    float state_vz = 0;

    float state_ax = 0;
    float state_ay = 0;
    float state_az = 0;

    float state_omegax = 0;
    float state_omegay = 0;
    float state_omegaz = 0;

    float state_latitude = 0;
    float state_longitude = 0;

    int32_t timeStamp_state = 0;
};

struct rocketStateData {
    FSM_State rocketState = STATE_INIT;
    int32_t timeStamp_RS = 0;
};


/**
 * @brief A struct to hold all of the data that could come from any of the sensors
 * 
 */
struct sensorDataStruct_t {
    //! data for lowGimu
    lowGData lowG_data;

    //! data for highGimu accel data (hg_x, hg_y, hg_z) 
    highGData highG_data;
    // int16_t pt1;
    // int16_t pt2;
    // int16_t pt3;

    //GPS DATA
    gpsData gps_data;

    // State variables
    stateData state_data;
    
    // Rocket State
    rocketStateData rocketState_data;

};

/**
 * @brief An enum to list all potential sensors.
 * 
 */
enum sensors {
    LOWG_IMU,
    HIGHG_IMU,
    GPS
};


#define FIFO_SIZE 1000
/**
 * @brief A struct to hold all info for ring buffers and mutexes used for data.
 * 
 */
struct datalogger_THD {
    //semaphore_t fifoData;
    SEMAPHORE_DECL(fifoData_lowG, 0);
    SEMAPHORE_DECL(fifoSpace_lowG, FIFO_SIZE);
    
    SEMAPHORE_DECL(fifoData_highG,0);
    SEMAPHORE_DECL(fifoSpace_highG, FIFO_SIZE);

    SEMAPHORE_DECL(fifoData_GPS,0);
    SEMAPHORE_DECL(fifoSpace_GPS, FIFO_SIZE);

    SEMAPHORE_DECL(fifoData_state,0);
    SEMAPHORE_DECL(fifoSpace_state, FIFO_SIZE);

    SEMAPHORE_DECL(fifoData_RS,0);
    SEMAPHORE_DECL(fifoSpace_RS, FIFO_SIZE);
    

    uint16_t fifoHead_lowG = 0;
    uint16_t fifoHead_highG = 0;
    uint16_t fifoHead_GPS = 0;
    uint16_t fifoHead_state = 0;
    uint16_t fifoHead_RS = 0;

    uint16_t fifoTail_all = 0;

    uint16_t bufferErrors_lowG = 0;
    uint16_t bufferErrors_highG = 0;
    uint16_t bufferErrors_GPS = 0;
    uint16_t bufferErrors_state = 0;
    uint16_t bufferErrors_RS = 0;

    MUTEX_DECL(dataMutex_lowG);
    MUTEX_DECL(dataMutex_highG);
    MUTEX_DECL(dataMutex_GPS);
    MUTEX_DECL(dataMutex_state);
    MUTEX_DECL(dataMutex_RS);

    sensorDataStruct_t fifoArray[FIFO_SIZE];

    sensorDataStruct_t current_data;

    File dataFile;
};



char* sd_file_namer(char* inputName, char* fileExtensionParam);

void logData(File* dataFile, sensorDataStruct_t* data);

#endif
