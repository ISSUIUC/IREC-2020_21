#ifndef MAIN_CPP
#define MAIN_CPP

#include <Arduino.h>
#include <ChRt.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <PWMServo.h>

#include "SparkFunLSM9DS1.h" //Low-G IMU Library
#include "KX134-1211.h" //High-G IMU Library
#include "ZOEM8Q0.hpp" //GPS Library
#include "hybridShared.h"
#include "acShared.h"
#include "dataLog.h"
#include "dataLog.cpp"
#include "thresholds.h"
#include "pins.h"

#include "sensors.cpp"
#include "servo.cpp"


datalogger_THD lowg_datalogger_THD_vars;
datalogger_THD highg_datalogger_THD_vars;
datalogger_THD gps_datalogger_THD_vars;


#define THREAD_DEBUG
//#define LOWGIMU_DEBUG
//#define HIGHGIMU_DEBUG
//#define GPS_DEBUG
//#define SERVO_DEBUG

//changed name to account for both high & lowG (logGData)
sensorDataStruct_t gpsSensorData;
sensorDataStruct_t lowgSensorData;
sensorDataStruct_t highgSensorData;

FSM_State rocketState = STATE_INIT;
fsm_struct rocketTimers;


KX134 highGimu;
LSM9DS1 lowGimu;
ZOEM8Q0 gps = ZOEM8Q0();

lowg_PNTR lowg_pntr;
highg_PNTR highg_pntr;
gps_PNTR gps_pntr;
servo_PNTR servo_pntr;



uint8_t mpu_data[70];

static THD_WORKING_AREA(gps_WA, 512);
static THD_WORKING_AREA(rocket_FSM_WA, 512);
static THD_WORKING_AREA(lowgIMU_WA, 512);
static THD_WORKING_AREA(highgIMU_WA, 512);
static THD_WORKING_AREA(servo_WA, 512);
static THD_WORKING_AREA(lowg_dataLogger_WA, 512);
static THD_WORKING_AREA(highg_dataLogger_WA, 512);
static THD_WORKING_AREA(gps_dataLogger_WA, 512);
static THD_WORKING_AREA(mpuComm_WA, 512);
static THD_WORKING_AREA(pulse_sender_WA, 512);
static THD_WORKING_AREA(pulse_receiver_WA, 512);

static THD_FUNCTION(mpuComm_THD, arg){
  //first 3 bytes of packet need to be iss
  (void)arg;

  Serial1.begin(115200); // Serial interface between MPU and MCU

  while (true) {

    #ifdef THREAD_DEBUG
      Serial.println("### mpuComm thread entrance");
    #endif

    //!locking data from sensorData struct
    chMtxLock(&lowg_datalogger_THD_vars.dataMutex);

    digitalWrite(LED_WHITE, HIGH);

    //write transmission code here
    unsigned i = 3; //because the first 3 indices are already set to be ISS 

    uint8_t* data = (uint8_t*) &lowgSensorData;
    mpu_data[0] = 0x49;
    mpu_data[1] = 0x53;
    mpu_data[2] = 0x53;

    for (; i < 3 + sizeof(lowgSensorData); i++) {
      mpu_data[i] = *data; //de-references to match data types, not sure if correct, might send only the first byte
      data++;
    }

    //TODO: Send rocket state too? Is there a mutex for rocket state?

    Serial1.write(mpu_data, sizeof(mpu_data));

    digitalWrite(LED_WHITE, LOW);
  
    //!Unlocking &dataMutex
    chMtxUnlock(&lowg_datalogger_THD_vars.dataMutex);

    chThdSleepMilliseconds(6); //Set equal sleep time as the other threads, can change  
  }
}





//
static THD_FUNCTION(rocket_FSM, arg){
  (void)arg;
  while(true){

    #ifdef THREAD_DEBUG
      Serial.println("### Rocket FSM thread entrance");
    #endif

    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // TODO - Acquire lock on data struct!
      chMtxLock(&lowg_datalogger_THD_vars.dataMutex);
      switch (rocketState) {
            case STATE_INIT:
                // TODO
            break;

            case STATE_IDLE:

                // If high acceleration is observed in z direction...
                //!locking mutex to get data from sensorData struct
                if(lowgSensorData.az > launch_az_thresh) {
                    rocketTimers.launch_time = chVTGetSystemTime();
                    rocketState = STATE_LAUNCH_DETECT;
                }
                //!unlocking &dataMutex mutex

            break;

            case STATE_LAUNCH_DETECT:

                //If the acceleration was too brief, go back to IDLE
                //!locking mutex to get data from sensorData struct
                if (lowgSensorData.az < launch_az_thresh) {
                    rocketState = STATE_IDLE;
                    break;
                }
                //!unlocking &dataMutex mutex

                // measure the length of the burn time (for hysteresis)
                rocketTimers.burn_timer =
                    chVTGetSystemTime() - rocketTimers.launch_time;

                // If the acceleration lasts long enough, boost is detected
                if (rocketTimers.burn_timer > launch_time_thresh) {
                    rocketState = STATE_BOOST;
                    digitalWrite(LED_RED, HIGH);
                }

            break;

            case STATE_BOOST:

            // If low acceleration in the Z direction...
            //!locking mutex to get data from sensorData struct
            if (lowgSensorData.az < coast_thresh) {
                rocketTimers.burnout_time = chVTGetSystemTime();
                rocketState = STATE_BURNOUT_DETECT;
            }
            //!unlocking &dataMutex mutex

            break;

            case STATE_BURNOUT_DETECT:

                //If the low acceleration was too brief, go back to BOOST
                //!locking mutex to get data from sensorData struct
                if (lowgSensorData.az > coast_thresh) {
                    rocketState = STATE_BOOST;
                    break;
                }
                //!unlocking &dataMutex mutex

                // measure the length of the coast time (for hysteresis)
                rocketTimers.coast_timer =
                    chVTGetSystemTime() - rocketTimers.burnout_time;

                // If the low acceleration lasts long enough, coast is detected
                if (rocketTimers.coast_timer > coast_time_thresh) {
                    rocketState = STATE_BOOST;
                }

            break;

            case STATE_COAST:
                // TODO
            break;

            case STATE_APOGEE_DETECT:
                // TODO
            break;

            case STATE_APOGEE:
                // TODO
            break;

            case STATE_DROGUE:
                // TODO
            break;

            case STATE_MAIN:
                // TODO
            break;

        }
        chMtxUnlock(&lowg_datalogger_THD_vars.dataMutex);

        

        chThdSleepMilliseconds(6); // FSM runs at 100 Hz
  }
}

bool write_val;
static virtual_timer_t write_vt;
static void toggle_write() {
  digitalWrite(/*INSERT PIN*/ 1, write_val);
  write_val = !write_val;
  chSysLockFromISR();
  chVTSetI(&write_vt, TIME_MS2I(100), toggle_write, NULL);
  chSysUnlockFromISR();
}
static THD_FUNCTION(pulse_sender_THD, arg) {
  write_val = 0;
  chVTObjectInit(&write_vt);
  chVTSet(&write_vt, TIME_MS2I(100), toggle_write, NULL);
}

bool read_val;
int num_missed;
static virtual_timer_t read_vt;
static void toggle_read() {
  int received = digitalRead(/*INSERT PIN*/ 1);
  if (received != read_val) {
    num_missed++;
  } else {
    num_missed = 0;
    read_val = !read_val;
  }
  if (num_missed > /*SOME THRESHOLD*/ 20) {
    /*ERROR HERE*/
    return;
  }
  chSysLockFromISR();
  chVTSetI(&read_vt, TIME_MS2I(100), toggle_read, NULL);
  chSysUnlockFromISR();
}
static THD_FUNCTION(pulse_receiver_THD, arg) {
  read_val = 0;
  chVTObjectInit(&read_vt);
  chVTSet(&read_vt, TIME_MS2I(100), toggle_read, NULL);
}

/**
 * @brief Starts all of the threads.
 * 
 */
void chSetup(){
  //added play_THD for creation

  chThdCreateStatic(rocket_FSM_WA, sizeof(rocket_FSM_WA), NORMALPRIO, rocket_FSM, NULL);
  chThdCreateStatic(gps_WA, sizeof(gps_WA), NORMALPRIO, gps_THD, &gps_pntr);
  chThdCreateStatic(lowgIMU_WA, sizeof(lowgIMU_WA), NORMALPRIO, lowgIMU_THD, &lowg_pntr);
  chThdCreateStatic(highgIMU_WA, sizeof(highgIMU_WA), NORMALPRIO, highgIMU_THD, &highg_pntr);
  chThdCreateStatic(servo_WA, sizeof(servo_WA), NORMALPRIO, servo_THD, &servo_pntr);
  chThdCreateStatic(lowg_dataLogger_WA, sizeof(lowg_dataLogger_WA), NORMALPRIO, dataLogger_THD, &lowg_datalogger_THD_vars);
  chThdCreateStatic(highg_dataLogger_WA, sizeof(highg_dataLogger_WA), NORMALPRIO, dataLogger_THD, &highg_datalogger_THD_vars);
  chThdCreateStatic(gps_dataLogger_WA, sizeof(gps_dataLogger_WA), NORMALPRIO, dataLogger_THD, &gps_datalogger_THD_vars);
  chThdCreateStatic(mpuComm_WA, sizeof(mpuComm_WA), NORMALPRIO, mpuComm_THD, NULL);

  chThdCreateStatic(pulse_sender_WA, sizeof(pulse_sender_WA), NORMALPRIO, pulse_sender_THD, NULL);
  chThdCreateStatic(pulse_receiver_WA, sizeof(pulse_receiver_WA), NORMALPRIO, pulse_receiver_THD, NULL);

  while(true);
}

/**
 * @brief Handles all configuration necessary before the threads start.
 * 
 */
void setup() {
  #if defined(THREAD_DEBUG) || defined(LOWGIMU_DEBUG) || defined(HIGHGIMU_DEBUG) || defined(GPS_DEBUG) || defined(SERVO_DEBUG)
    Serial.begin(115200);
    while (!Serial) {}
  #endif

  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_ORANGE, OUTPUT);
  pinMode(LED_WHITE, OUTPUT);

  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_ORANGE, HIGH);

  //TODO: Don't forget this
  Serial.println("------------------------------------------------");

  lowg_pntr.lowGimuPointer = &lowGimu;
  lowg_pntr.sensorDataPointer = &lowgSensorData;
  lowg_pntr.rocketStatePointer = &rocketState;
  lowg_pntr.dataloggerTHDVarsPointer = &lowg_datalogger_THD_vars;

  highg_pntr.highGimuPointer = &highGimu;
  highg_pntr.sensorDataPointer = &highgSensorData;
  highg_pntr.rocketStatePointer = &rocketState;
  highg_pntr.dataloggerTHDVarsPointer = &highg_datalogger_THD_vars;

  gps_pntr.GPSimuPointer = &gps;
  gps_pntr.sensorDataPointer = &gpsSensorData;
  gps_pntr.rocketStatePointer = &rocketState;
  gps_pntr.dataloggerTHDVarsPointer = &gps_datalogger_THD_vars;

  servo_pntr.rocketStatePointer = &rocketState;
  servo_pntr.lowgSensorDataPointer = &lowgSensorData;
  servo_pntr.lowgDataloggerTHDVarsPointer = &lowg_datalogger_THD_vars;

  //lowGimu setup
  if (lowGimu.beginSPI(LSM9DS1_AG_CS, LSM9DS1_M_CS) == false) // note, we need to sent this our CS pins (defined above)
  {
    digitalWrite(LED_RED, HIGH);
    Serial.println("Failed to communicate with LSM9DS1. Stalling Program");
    while (true);
  }

  lowGimu.setAccelScale(16);
  
  //GPS Setup
 	gps.beginSPI(ZOEM8Q0_CS);

  // Setup sensor_type for each sensor
  lowg_datalogger_THD_vars.sensor_type = LOWG_IMU;
  highg_datalogger_THD_vars.sensor_type = HIGHG_IMU;
  gps_datalogger_THD_vars.sensor_type = GPS;

  //SD Card Setup
  if(SD.begin(BUILTIN_SDCARD)){

    char file_extension[6] = ".dat";

    char data_name[16] = "data";
    lowg_datalogger_THD_vars.dataFile = SD.open(sd_file_namer(data_name, file_extension),O_CREAT | O_WRITE | O_TRUNC);
    highg_datalogger_THD_vars.dataFile = lowg_datalogger_THD_vars.dataFile;
    gps_datalogger_THD_vars.dataFile = lowg_datalogger_THD_vars.dataFile;
    lowg_datalogger_THD_vars.dataFile.println("ax,ay,az,gx,gy,gz,mx,my,mz,hg_ax,hg_ay,hg_az,latitude,longitude,altitude,GPS Lock,rocketState,timeStamp");
    // Serial.println(lowg_datalogger_THD_vars.dataFile.name());

    
  }
  else {
    digitalWrite(LED_RED, HIGH);
    Serial.println("SD Begin Failed. Stalling Program");
    while(true);
  }

  //Servo Setup
  servo_cw.attach(BALL_VALVE_1_PIN, 770, 2250); //TODO: MAKE SURE TO CHANGE PINS
  servo_ccw.attach(BALL_VALVE_2_PIN, 770, 2250);

  Serial.println("Starting ChibiOS");
  chBegin(chSetup);
  while(true);

  
}

void loop() {
  // not used
}

#endif