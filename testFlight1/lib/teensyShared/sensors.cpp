#ifndef SENSORS_CPP
#define SENSORS_CPP

#include <Arduino.h>
#include <ChRt.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <PWMServo.h>
//TODO remove
#include "main.cpp"

// #include "SparkFunLSM9DS1.h" //Low-G IMU Library
// #include "KX134-1211.h" //High-G IMU Library
// #include "ZOEM8Q0.hpp" //GPS Library
#include "hybridShared.h"
#include "acShared.h"
// #include "dataLog.h"
// #include "dataLog.cpp"
// #include "thresholds.h"
#include "pins.h"
#include "sensors.h"


/**
 * @brief Construct a new thd function object to handle data collection from the low-g IMU.
 * 
 * @param arg Contains pointers to various objects needed by the low-g IMU.
 * 
 */
static THD_FUNCTION(lowgIMU_THD, arg) {
  struct pointers *pointer_struct = (struct pointers *)arg;
  while(true){

    #ifdef THREAD_DEBUG
      // Serial.println("### Low G IMU thread entrance");
    #endif

    chSysLock();
    pointer_struct->lowGimuPointer->readAccel();
    pointer_struct->lowGimuPointer->readGyro();
    pointer_struct->lowGimuPointer->readMag();
    chSysUnlock();

    chMtxLock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_lowG);
    pointer_struct->sensorDataPointer->lowG_data.timeStamp_lowG = chVTGetSystemTime();

    //acceleration in Gs
    pointer_struct->sensorDataPointer->lowG_data.ax = pointer_struct->lowGimuPointer->calcAccel(pointer_struct->lowGimuPointer->ax);
    pointer_struct->sensorDataPointer->lowG_data.ay = pointer_struct->lowGimuPointer->calcAccel(pointer_struct->lowGimuPointer->ay);
    pointer_struct->sensorDataPointer->lowG_data.az = pointer_struct->lowGimuPointer->calcAccel(pointer_struct->lowGimuPointer->az); //There was a minus here. We don't know why that did that
    //rotational speed in degrees per second
    pointer_struct->sensorDataPointer->lowG_data.gx = pointer_struct->lowGimuPointer->calcGyro(pointer_struct->lowGimuPointer->gx);
    pointer_struct->sensorDataPointer->lowG_data.gy = pointer_struct->lowGimuPointer->calcGyro(pointer_struct->lowGimuPointer->gy);
    pointer_struct->sensorDataPointer->lowG_data.gz = pointer_struct->lowGimuPointer->calcGyro(pointer_struct->lowGimuPointer->gz);
    //magnatometer data in gauss 
    pointer_struct->sensorDataPointer->lowG_data.mx = pointer_struct->lowGimuPointer->calcMag(pointer_struct->lowGimuPointer->mx);
    pointer_struct->sensorDataPointer->lowG_data.my = pointer_struct->lowGimuPointer->calcMag(pointer_struct->lowGimuPointer->my);
    pointer_struct->sensorDataPointer->lowG_data.mz = pointer_struct->lowGimuPointer->calcMag(pointer_struct->lowGimuPointer->mz);
    //!Unlocking &dataMutex
    chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_lowG);

    #ifdef LOWGIMU_DEBUG
      Serial.println("------------- LOW-G THREAD ---------------");
      Serial.print(pointer_struct->sensorDataPointer->lowG_data.ax);
      Serial.print(", ");
      Serial.print(pointer_struct->sensorDataPointer->lowG_data.ay);
      Serial.print(", ");
      Serial.print(pointer_struct->sensorDataPointer->lowG_data.az);
      Serial.print(", ");
      Serial.print(pointer_struct->sensorDataPointer->lowG_data.gx);
      Serial.print(", ");
      Serial.print(pointer_struct->sensorDataPointer->lowG_data.gy);
      Serial.print(", ");
      Serial.print(pointer_struct->sensorDataPointer->lowG_data.gz);
      Serial.print(", ");
      Serial.print(pointer_struct->sensorDataPointer->lowG_data.mx);
      Serial.print(", ");
      Serial.print(pointer_struct->sensorDataPointer->lowG_data.my);
      Serial.print(", ");
      Serial.print(pointer_struct->sensorDataPointer->lowG_data.mz);
      Serial.print(", ");
    #endif 

    // logData(&dataFile, &sensorData, rocketState);
    // add the data to the buffer here!
    if (chSemWaitTimeout(&pointer_struct->dataloggerTHDVarsPointer.fifoSpace_lowG, TIME_IMMEDIATE) != MSG_OK) {
        pointer_struct->dataloggerTHDVarsPointer.bufferErrors_lowG++;
        digitalWrite(LED_BUILTIN, HIGH);
        continue;
    }
    chMtxLock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_lowG);
    pointer_struct->dataloggerTHDVarsPointer.fifoArray[pointer_struct->dataloggerTHDVarsPointer.fifoHead_lowG].lowG_data = pointer_struct->sensorDataPointer->lowG_data;
    pointer_struct->dataloggerTHDVarsPointer.bufferErrors_lowG = 0;
    pointer_struct->dataloggerTHDVarsPointer.fifoHead_lowG = pointer_struct->dataloggerTHDVarsPointer.fifoHead_lowG < (FIFO_SIZE - 1) ? pointer_struct->dataloggerTHDVarsPointer.fifoHead_lowG + 1 : 0;
    chSemSignal(&pointer_struct->dataloggerTHDVarsPointer.fifoData_lowG);

    //!Unlocking &dataMutex
    chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_lowG);

    chThdSleepMilliseconds(6);
  }
}

/**
 * @brief Construct a new thd function object to handle data collection from the GPS.
 * 
 * @param arg Contains pointers to various objects needed by the GPS.
 * 
 */
static THD_FUNCTION(gps_THD, arg){
  struct pointers *pointer_struct = (struct pointers *)arg;
  while(true){
    
    #ifdef THREAD_DEBUG
      // Serial.println("### GPS thread entrance");
    #endif

    chSysLock();
    pointer_struct->GPSPointer->update_data();
    chSysUnlock();

    chMtxLock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_GPS);
    

    pointer_struct->sensorDataPointer->gps_data.timeStamp_GPS = chVTGetSystemTime();
    //Have the availability to wait until a lock is aquired with gps.get_position_lock();
    pointer_struct->sensorDataPointer->gps_data.latitude = pointer_struct->GPSPointer->get_latitude();
    pointer_struct->sensorDataPointer->gps_data.longitude = pointer_struct->GPSPointer->get_longitude();
    pointer_struct->sensorDataPointer->gps_data.altitude = pointer_struct->GPSPointer->get_altitude();
    pointer_struct->sensorDataPointer->gps_data.posLock = pointer_struct->GPSPointer->get_position_lock();
    //!Unlocking &dataMutex
    chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_GPS);

    

    if(pointer_struct->sensorDataPointer->gps_data.posLock == true){
      digitalWrite(LED_ORANGE, HIGH);
    }else{
      digitalWrite(LED_ORANGE, LOW);
    }

    #ifdef GPS_DEBUG
      bool position_lock = pointer_struct->GPSPointer->get_position_lock();
      if (position_lock) {
        Serial.println("POSITION LOCK!");
        Serial.println("GPS Data: ");
        Serial.print("Latitude: ");
        Serial.println(pointer_struct->sensorDataPointer->gps_data.latitude);
        Serial.print("Longitude: ");
        Serial.println(pointer_struct->sensorDataPointer->gps_data.longitude);
        Serial.print("Altitude: ");
        Serial.println(pointer_struct->sensorDataPointer->gps_data.altitude);
      } else {
        Serial.println("Searching...");
        Serial.print("Latitude: ");
        Serial.println(pointer_struct->sensorDataPointer->gps_data.latitude);
        Serial.print("Longitude: ");
        Serial.println(pointer_struct->sensorDataPointer->gps_data.longitude);
        Serial.print("Altitude: ");
        Serial.println(pointer_struct->sensorDataPointer->gps_data.altitude);
	      Serial.println("");
      }
    #endif

    // add the data to the buffer here!
    if (chSemWaitTimeout(&pointer_struct->dataloggerTHDVarsPointer.fifoSpace_GPS, TIME_IMMEDIATE) != MSG_OK) {
        pointer_struct->dataloggerTHDVarsPointer.bufferErrors_GPS++;
        digitalWrite(LED_BUILTIN, HIGH);
        continue;
    }
    chMtxLock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_GPS);
    pointer_struct->dataloggerTHDVarsPointer.fifoArray[pointer_struct->dataloggerTHDVarsPointer.fifoHead_GPS].gps_data = pointer_struct->sensorDataPointer->gps_data;
    pointer_struct->dataloggerTHDVarsPointer.bufferErrors_GPS = 0;
    pointer_struct->dataloggerTHDVarsPointer.fifoHead_GPS = pointer_struct->dataloggerTHDVarsPointer.fifoHead_GPS < (FIFO_SIZE - 1) ? pointer_struct->dataloggerTHDVarsPointer.fifoHead_GPS + 1 : 0;
    chSemSignal(&pointer_struct->dataloggerTHDVarsPointer.fifoData_GPS);
 

    //!Unlocking &dataMutex
    chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_GPS);

    #ifdef THREAD_DEBUG
      // Serial.println("### GPS thread exit");
    #endif

    chThdSleepMilliseconds(6); // Sensor DAQ @ ~100 Hz
  }
}

/**
 * @brief Construct a new thd function object to handle data collection from the high-g IMU.
 * 
 * @param arg Contains pointers to the various objects needed by the high-g IMU.
 * 
 */
static THD_FUNCTION(highgIMU_THD, arg){
  struct pointers *pointer_struct = (struct pointers *)arg;
  while(true){

    #ifdef THREAD_DEBUG
      // Serial.println("### High G IMU thread entrance");
    #endif

    

    chSysLock();
    pointer_struct->highGimuPointer->update_data();
    chSysUnlock();

    chMtxLock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_highG);

    pointer_struct->sensorDataPointer->highG_data.timeStamp_highG = chVTGetSystemTime();

    //addition for highG IMU
    pointer_struct->sensorDataPointer->highG_data.hg_ax = pointer_struct->highGimuPointer->get_x_gforce();
    pointer_struct->sensorDataPointer->highG_data.hg_ay = pointer_struct->highGimuPointer->get_y_gforce();
    pointer_struct->sensorDataPointer->highG_data.hg_az = pointer_struct->highGimuPointer->get_z_gforce();

    chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_highG);

    #ifdef HIGHGIMU_DEBUG
      Serial.println("------------- HIGH-G THREAD ---------------");
      //high g data
      Serial.print(pointer_struct->sensorDataPointer->highG_data.hg_ax);
      Serial.print(", ");
      Serial.print(pointer_struct->sensorDataPointer->highG_data.hg_ay);
      Serial.print(", ");
      Serial.println(pointer_struct->sensorDataPointer->highG_data.hg_az);
    #endif

    // add the data to the buffer here!
    if (chSemWaitTimeout(&pointer_struct->dataloggerTHDVarsPointer.fifoSpace_highG, TIME_IMMEDIATE) != MSG_OK) {
        pointer_struct->dataloggerTHDVarsPointer.bufferErrors_highG++;
        digitalWrite(LED_BUILTIN, HIGH);
        continue;
    }
    chMtxLock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_highG);
    pointer_struct->dataloggerTHDVarsPointer.fifoArray[pointer_struct->dataloggerTHDVarsPointer.fifoHead_highG].highG_data = pointer_struct->sensorDataPointer->highG_data;
    pointer_struct->dataloggerTHDVarsPointer.bufferErrors_highG = 0;
    pointer_struct->dataloggerTHDVarsPointer.fifoHead_highG = pointer_struct->dataloggerTHDVarsPointer.fifoHead_highG < (FIFO_SIZE - 1) ? pointer_struct->dataloggerTHDVarsPointer.fifoHead_highG + 1 : 0;
    chSemSignal(&pointer_struct->dataloggerTHDVarsPointer.fifoData_highG);


    chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_highG);

    chThdSleepMilliseconds(6);
  }
}
#endif