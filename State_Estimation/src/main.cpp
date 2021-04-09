/* This is going to be used to experiment with different state estimation
 * methods. For now the first method used will be to use an existing filter
 * that estimates orientation, and then subtracts the gravity to get the acceleration.
 * Those are then integrated to get not-so-accurate velocity and position, so we have
 * a complete inertial solution. After that a Kalman filter takes in GNSS data
 * to calculate the corrections to the inertial solution to get a final integrated solution
 * Madgwick reference: https://github.com/PaulStoffregen/MadgwickAHRS/blob/master/examples/Visualize101/Visualize101.ino
 * Code by IREC Avionics 2021
 */

#include <Arduino.h>
#include "SparkFunLSM9DS1.h" //Low-G IMU Library
#include <MadgwickAHRS.h> //Madgwick library to get orientation


//define magnetometer chip select pin
#define LSM9DS1_M_CS 37
//define accel/gyro chip select pin
#define LSM9DS1_AG_CS 36
//Rate at which stuff is calculated
#define RATE 25

Madgwick filter;
LSM9DS1 lowGimu;

unsigned long microsPerReading, microsPrevious;


void setup() {
  Serial.begin(9600);

  //setup lowg IMU
  if (lowGimu.beginSPI(LSM9DS1_AG_CS, LSM9DS1_M_CS) == false) // note, we need to sent this our CS pins (defined above)
  {
    Serial.println("Failed to communicate with LSM9DS1. Stalling Program");
    while (true);
  }

  lowGimu.setAccelScale(16);
  lowGimu.setGyroODR(RATE);   // setting accel and gyro rates to be the same as filter
  lowGimu.setAccelODR(RATE); // setting accel and gyro rates to be the same as filter

  filter.begin(RATE); //set rate to be same as rate of IMU
}

void loop() {
  float roll, pitch, yaw;
  unsigned long microsNow; //For pacekeeping I believe


  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {
    lowGimu.readAccel();
    lowGimu.readGyro();
    // Get accelerometer readings in g
    float ax = lowGimu.calcAccel(lowGimu.ax);
    float ay = lowGimu.calcAccel(lowGimu.ay);
    float az = lowGimu.calcAccel(lowGimu.az);

    // Get gyro readings in degrees per second
    float gx = lowGimu.calcGyro(lowGimu.gx);
    float gy = lowGimu.calcGyro(lowGimu.gy);
    float gz = lowGimu.calcGyro(lowGimu.gz);


    filter.updateIMU(gx, gy, gz, ax, ay, az);
    // print the yaw, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    yaw = filter.getYaw();
    Serial.print("Orientation: ");
    Serial.print(yaw);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);

    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;

  }

}