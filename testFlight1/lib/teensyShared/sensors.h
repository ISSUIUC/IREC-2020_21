#ifndef SENSORS_H
#define SENSORS_H

#include "SparkFunLSM9DS1.h" //Low-G IMU Library
#include "KX134-1211.h" //High-G IMU Library
#include "ZOEM8Q0.hpp" //GPS Library

#include "dataLog.h"


struct lowg_PNTR {
    LSM9DS1 *lowGimuPointer;

    sensorDataStruct_t *sensorDataPointer;

    FSM_State *rocketStatePointer;

    datalogger_THD *dataloggerTHDVarsPointer;
};

struct highg_PNTR {
    KX134 *highGimuPointer;

    sensorDataStruct_t *sensorDataPointer;

    FSM_State *rocketStatePointer;

    datalogger_THD *dataloggerTHDVarsPointer;
};

struct gps_PNTR {
    ZOEM8Q0 *GPSimuPointer;

    sensorDataStruct_t *sensorDataPointer;

    FSM_State *rocketStatePointer;

    datalogger_THD *dataloggerTHDVarsPointer;
};

#endif