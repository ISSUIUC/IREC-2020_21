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
#include "thresholds.h"
#include "pins.h"

//!Creating mutex to prevent overlapping reads from play_THD and THD_FUNCTION
//!for reading sensorData struct
static MUTEX_DECL(dataMutex);

#define THREAD_DEBUG
//#define LOWGIMU_DEBUG
//#define HIGHGIMU_DEBUG
//#define GPS_DEBUG
//#define SERVO_DEBUG

//changed name to account for both high & lowG (logGData)
dataStruct_t sensorData;

FSM_State rocketState = STATE_INIT;
fsm_struct rocketTimers;

File dataFile;

KX134 highGimu;
LSM9DS1 lowGimu;
ZOEM8Q0 gps = ZOEM8Q0();

PWMServo servo_cw; //Servo that controlls roll in the clockwise direction
PWMServo servo_ccw; //Servo that controlls roll in the counter clockwise direction

float flap_drag;
float native_drag;

void round_off_angle(int &value) {
  if (value > 180) {
    value = 180;
  }
  if (value < 0) {
    value = 0;
  }
}

uint8_t mpu_data[70];

static THD_WORKING_AREA(gps_WA, 512);
static THD_WORKING_AREA(rocket_FSM_WA, 512);
static THD_WORKING_AREA(lowgIMU_WA, 512);
static THD_WORKING_AREA(highgIMU_WA, 512);
static THD_WORKING_AREA(servo_WA, 512);
static THD_WORKING_AREA(mpuComm_WA, 512);

static THD_FUNCTION(mpuComm_THD, arg){
  //first 3 bytes of packet need to be iss
  (void)arg;

  Serial1.begin(115200); // Serial interface between MPU and MCU

  while (true) {

    #ifdef THREAD_DEBUG
      Serial.println("### mpuComm thread entrance");
    #endif

    //!locking data from sensorData struct
    chMtxLock(&dataMutex);

    digitalWrite(LED_WHITE, HIGH);

    //write transmission code here
    unsigned i = 3; //because the first 3 indices are already set to be ISS 

    uint8_t* data = (uint8_t*) &sensorData;
    mpu_data[0] = 0x49;
    mpu_data[1] = 0x53;
    mpu_data[2] = 0x53;

    for (; i < 3 + sizeof(sensorData); i++) {
      mpu_data[i] = *data; //de-references to match data types, not sure if correct, might send only the first byte
      data++;
    }

    //TODO: Send rocket state too? Is there a mutex for rocket state?

    Serial1.write(mpu_data, sizeof(mpu_data));

    digitalWrite(LED_WHITE, LOW);
  
    //!Unlocking &dataMutex
    chMtxUnlock(&dataMutex);

    chThdSleepMilliseconds(6); //Set equal sleep time as the other threads, can change  
  }
}

static THD_FUNCTION(gps_THD, arg){
  (void)arg;
  while(true){
    
    #ifdef THREAD_DEBUG
      Serial.println("### GPS thread entrance");
    #endif

    //!locking data from sensorData struct
    chMtxLock(&dataMutex);

    sensorData.timeStamp = chVTGetSystemTime();

    chSysLock();
    gps.update_data();
    chSysUnlock();

    //Have the availability to wait until a lock is aquired with gps.get_position_lock();
    sensorData.latitude = gps.get_latitude();
    sensorData.longitude = gps.get_longitude();
    sensorData.altitude = gps.get_altitude();
    sensorData.posLock = gps.get_position_lock();
    

    if(sensorData.posLock == true){
      digitalWrite(LED_ORANGE, HIGH);
    }else{
      digitalWrite(LED_ORANGE, LOW);
    }

    #ifdef GPS_DEBUG
      bool position_lock = gps.get_position_lock();
      if (position_lock) {
        Serial.println("POSITION LOCK!");
        Serial.println("GPS Data: ");
        Serial.print("Latitude: ");
        Serial.println(sensorData.latitude);
        Serial.print("Longitude: ");
        Serial.println(sensorData.longitude);
        Serial.print("Altitude: ");
        Serial.println(sensorData.altitude);
      } else {
        Serial.println("Searching...");
        Serial.print("Latitude: ");
        Serial.println(sensorData.latitude);
        Serial.print("Longitude: ");
        Serial.println(sensorData.longitude);
        Serial.print("Altitude: ");
        Serial.println(sensorData.altitude);
	      Serial.println("");
      }
    #endif

    logData(&dataFile, &sensorData, rocketState);

    //!Unlocking &dataMutex
    chMtxUnlock(&dataMutex);  

    chThdSleepMilliseconds(6); // Sensor DAQ @ ~100 Hz
  }
}


static THD_FUNCTION(lowgIMU_THD, arg) {
  (void)arg;
  while(true){

    #ifdef THREAD_DEBUG
      Serial.println("### Low G IMU thread entrance");
    #endif

    chMtxLock(&dataMutex);

    chSysLock();
    lowGimu.readAccel();
    lowGimu.readGyro();
    lowGimu.readMag();
    chSysUnlock();

    //acceleration in Gs
    sensorData.ax = lowGimu.calcAccel(lowGimu.ax);
    sensorData.ay = lowGimu.calcAccel(lowGimu.ay);
    sensorData.az = lowGimu.calcAccel(lowGimu.az); //There was a minus here. We don't know why that did that
    //rotational speed in degrees per second
    sensorData.gx = lowGimu.calcGyro(lowGimu.gx);
    sensorData.gy = lowGimu.calcGyro(lowGimu.gy);
    sensorData.gz = lowGimu.calcGyro(lowGimu.gz);
    //magnatometer data in gauss 
    sensorData.mx = lowGimu.calcMag(lowGimu.mx);
    sensorData.my = lowGimu.calcMag(lowGimu.my);
    sensorData.mz = lowGimu.calcMag(lowGimu.mz);

    #ifdef LOWGIMU_DEBUG
      Serial.println("------------- LOW-G THREAD ---------------");
      Serial.print(sensorData.ax);
      Serial.print(", ");
      Serial.print(sensorData.ay);
      Serial.print(", ");
      Serial.print(sensorData.az);
      Serial.print(", ");
      Serial.print(sensorData.gx);
      Serial.print(", ");
      Serial.print(sensorData.gy);
      Serial.print(", ");
      Serial.print(sensorData.gz);
      Serial.print(", ");
      Serial.print(sensorData.mx);
      Serial.print(", ");
      Serial.print(sensorData.my);
      Serial.print(", ");
      Serial.print(sensorData.mz);
      Serial.print(", ");
    #endif

    chMtxUnlock(&dataMutex);

    chThdSleepMilliseconds(6);
  }
}


static THD_FUNCTION(highgIMU_THD, arg){
  (void)arg;
  while(true){

    #ifdef THREAD_DEBUG
      Serial.println("### High G IMU thread entrance");
    #endif

    chMtxLock(&dataMutex);

    chSysLock();
    highGimu.update_data();
    chSysUnlock();

    //addition for highG IMU
    sensorData.hg_ax = highGimu.get_x_gforce();
    sensorData.hg_ay = highGimu.get_y_gforce();
    sensorData.hg_az = highGimu.get_z_gforce();

    #ifdef HIGHGIMU_DEBUG
      Serial.println("------------- HIGH-G THREAD ---------------");
      //high g data
      Serial.print(sensorData.hg_ax);
      Serial.print(", ");
      Serial.print(sensorData.hg_ay);
      Serial.print(", ");
      Serial.println(sensorData.hg_az);
    #endif

    chMtxUnlock(&dataMutex);

    chThdSleepMilliseconds(6);
  }
}


static THD_FUNCTION(rocket_FSM, arg){
  (void)arg;
  while(true){

    #ifdef THREAD_DEBUG
      Serial.println("### Rocket FSM thread entrance");
    #endif

    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // TODO - Acquire lock on data struct!
      chMtxLock(&dataMutex);
      switch (rocketState) {
            case STATE_INIT:
                // TODO
            break;

            case STATE_IDLE:

                // If high acceleration is observed in z direction...
                //!locking mutex to get data from sensorData struct
                if(sensorData.az > launch_az_thresh) {
                    rocketTimers.launch_time = chVTGetSystemTime();
                    rocketState = STATE_LAUNCH_DETECT;
                }
                //!unlocking &dataMutex mutex

            break;

            case STATE_LAUNCH_DETECT:

                //If the acceleration was too brief, go back to IDLE
                //!locking mutex to get data from sensorData struct
                if (sensorData.az < launch_az_thresh) {
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
            if (sensorData.az < coast_thresh) {
                rocketTimers.burnout_time = chVTGetSystemTime();
                rocketState = STATE_BURNOUT_DETECT;
            }
            //!unlocking &dataMutex mutex

            break;

            case STATE_BURNOUT_DETECT:

                //If the low acceleration was too brief, go back to BOOST
                //!locking mutex to get data from sensorData struct
                if (sensorData.az > coast_thresh) {
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
        chMtxUnlock(&dataMutex);

        chThdSleepMilliseconds(6); // FSM runs at 100 Hz
  }
}

static THD_FUNCTION(servo_THD, arg){
  (void)arg;
  while(true){

    #ifdef THREAD_DEBUG
      Serial.println("### Servo thread entrance");
    #endif
    
    int ccw_angle = 90;
    int cw_angle = 90;
    bool active_control = false;

    chMtxLock(&dataMutex);

    switch(rocketState) {
      case STATE_INIT :
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
      cw_angle = sensorData.gz;
      ccw_angle = sensorData.gz;

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

    chMtxUnlock(&dataMutex);
    chThdSleepMilliseconds(6); // FSM runs at 100 Hz
  }

}
void chSetup(){
  //added play_THD for creation

  chThdCreateStatic(rocket_FSM_WA, sizeof(rocket_FSM_WA), NORMALPRIO, rocket_FSM, NULL);
  chThdCreateStatic(gps_WA, sizeof(gps_WA), NORMALPRIO, gps_THD, NULL);
  chThdCreateStatic(lowgIMU_WA, sizeof(lowgIMU_WA), NORMALPRIO, lowgIMU_THD, NULL);
  chThdCreateStatic(highgIMU_WA, sizeof(highgIMU_WA), NORMALPRIO, highgIMU_THD, NULL);
  chThdCreateStatic(servo_WA, sizeof(servo_WA), NORMALPRIO, servo_THD, NULL);
  chThdCreateStatic(mpuComm_WA, sizeof(mpuComm_WA), NORMALPRIO, mpuComm_THD, NULL);
  while(true);
}


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

  //SD Card Setup
  if(SD.begin(BUILTIN_SDCARD)){
    init_dataLog(&dataFile);
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
