#ifndef SERVO_CPP
#define SERVO_CPP

#include <Arduino.h>
#include <ChRt.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <PWMServo.h>
//TODO remove
#include "main.cpp"

#include "SparkFunLSM9DS1.h" //Low-G IMU Library
#include "KX134-1211.h" //High-G IMU Library
#include "ZOEM8Q0.hpp" //GPS Library
#include "hybridShared.h"
#include "acShared.h"
#include "dataLog.h"
#include "dataLog.cpp"
#include "thresholds.h"
#include "pins.h"
#include "servo.h"
#include "sensors.h"

PWMServo servo_cw; //Servo that controlls roll in the clockwise direction
PWMServo servo_ccw; //Servo that controlls roll in the counter clockwise direction

float flap_drag;
float native_drag;
float K_gain;

/**
 * @brief A function to keep the value sent to the servo between 0 and 180 degrees.
 * 
 * @param value The anglevalue determined by the control algorithm.
 */
void round_off_angle(int &value) {
  if (value > 180) {
    value = 180;
  }
  if (value < 0) {
    value = 0;
  }
}

/**
 * @brief A function to convert the control length input to the servo angle
 * 
 * @param len The length input given by the control system
 */
float len2ang(float len) {
  ///convert from the length input to the angle required for that length
  //this should be based on emperical testing
  //TODO
  return 0;
}

/**
 * @brief Construct a new thd function object to control the servo.
 * 
 * @param arg A struct containing pointers to objects needed to run the thread.
 * 
 */
static THD_FUNCTION(servo_THD, arg){
  struct pointers *pointer_struct = (struct pointers *)arg;
  bool active_control = false;
  while(true){
    #ifdef THREAD_DEBUG
      // Serial.println("### Servo thread entrance");
    #endif
    
    int ccw_angle = -90; // Give different starting values 
    int cw_angle = 90;
    active_control = false;

    sensorDataStruct_t *current_data;
    chMtxLock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_state);
    current_data = pointer_struct->sensorDataPointer->state_data;

    //Get state data
    float roll = current_data.state_omegax;
    float vx = current_data.state_vx;
    float x = current_data.state_x;

    chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_state);
    float u[1][2] = {{}}; //Inputs to the servo

    chMtxLock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_RS);
    FSM_State currentRocketState = pointer_struct->sensorDataPointer->rocketState_data.rocketState;
    chMtxUnlock(&pointer_struct->dataloggerTHDVarsPointer.dataMutex_RS);

    switch(currentRocketState) {
      case STATE_INIT:
        active_control = true;
        break;
      case STATE_IDLE:
        active_control = true;
        break;
      case STATE_LAUNCH_DETECT :
        active_control = true;
        break;
      case STATE_BOOST :
        active_control = false;
        break;
      case STATE_COAST :
        active_control = true;
        break;
      case STATE_APOGEE_DETECT :
        active_control = false;
        break;
      default :
        active_control = false;
      break;
    }
    // turns active control off if not in takeoff/coast sequence
    if (active_control) {
      //u = -Kx
      //length to angle conversion
      ccw_angle = len2ang(u[0][0]);
      cw_angle = len2ang(u[1][0]);
    } else {
      //Turns active control off if not in coast state.
      cw_angle = 0;
      ccw_angle = 0;
    }
    round_off_angle(cw_angle);
    round_off_angle(ccw_angle);
    servo_cw.write(cw_angle);
    servo_ccw.write(ccw_angle); 
    
    #ifdef SERVO_DEBUG
      Serial.print("\nclockwise: ");
      Serial.print(cw_angle);
      Serial.print(" counterclockwise: ");
      Serial.print(ccw_angle);
    #endif

    
    chThdSleepMilliseconds(6); // FSM runs at 100 Hz
  }

}
#endif