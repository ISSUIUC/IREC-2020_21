#ifndef SENSORS_H
#define SENSORS_H

#include "SparkFunLSM9DS1.h" //Low-G IMU Library
#include "KX134-1211.h" //High-G IMU Library
#include "ZOEM8Q0.hpp" //GPS Library

#include "dataLog.h"

struct pointers {
    LSM9DS1 *lowGimuPointer;
    KX134 *highGimuPointer;
    ZOEM8Q0 *GPSPointer;

    sensorDataStruct_t *sensorDataPointer;

    datalogger_THD *dataloggerTHDVarsPointer;
};

/**
 * @brief A struct containing pointers needed by the lowgIMU_THD
 * 
 */
struct lowg_PNTR {
    LSM9DS1 *lowGimuPointer;

    sensorDataStruct_t *sensorDataPointer;

    FSM_State *rocketStatePointer;

    datalogger_THD *dataloggerTHDVarsPointer;
};

/**
 * @brief A struct containing pointers needed by the highgIMU_THD
 * 
 */
struct highg_PNTR {
    KX134 *highGimuPointer;

    sensorDataStruct_t *sensorDataPointer;

    FSM_State *rocketStatePointer;

    datalogger_THD *dataloggerTHDVarsPointer;
};

/**
 * @brief A struct containing pointers needed by the gps_THD
 * 
 */
struct gps_PNTR {
    ZOEM8Q0 *GPSimuPointer;

    sensorDataStruct_t *sensorDataPointer;

    FSM_State *rocketStatePointer;

    datalogger_THD *dataloggerTHDVarsPointer;
};

#endif