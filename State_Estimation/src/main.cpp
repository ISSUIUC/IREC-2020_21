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
float vx = 0, vy = 0, vz = 0; //Inertial frame velocities
float x = 0, y = 0, z = 0; //inertial frame positions


//Co-ordinate transformation matrix from body to inertial 
void body_to_inertial(float C_body_to_init[3][3], float q0, float q1, float q2, float q3) {
  // Matrix expression from Groves, pg. 41
  C_body_to_init[0][0] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
  C_body_to_init[1][1] = q0*q0 - q1*q1 + q2*q2 - q3*q3;
  C_body_to_init[2][2] = q0*q0 + q1*q1 - q2*q2 + q3*q3;
  C_body_to_init[0][1] = 2*(q1*q2 - q3*q0);
  C_body_to_init[1][0] = 2*(q1*q2 + q3*q0);
  C_body_to_init[0][2] = 2*(q1*q3 + q2*q0);
  C_body_to_init[2][0] = 2*(q1*q3 - q2*q0);
  C_body_to_init[1][2] = 2*(q2*q3 - q1*q0);
  C_body_to_init[2][1] = 2*(q2*q3 + q1*q0);
}

//Write a function for converting specific force to inertial frame accel
void specf_to_ai(float *aix, float* aiy, float *aiz, float ax, float ay, float az, float rotate[3][3]) {
  float specfi[3] = {}; //accelerations in reference frame
  for (int i = 0; i < 2; i++) {
    specfi[i] = rotate[i][0]*ax + rotate[i][1]*ay + rotate[i][2]*az;
    // Converting g's into m/s^2
    specfi[i] *= 9.81;
  }
  //Adding gravity:
  *aix = specfi[0] - 9.81;
  *aiy = specfi[1];
  *aiz = specfi[2];
}


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
  lowGimu.setMagODR(RATE);

  filter.begin(RATE); //set rate to be same as rate of IMU
}

void loop() {
  float roll, pitch, yaw;
  float q0, q1, q2, q3;
  unsigned long microsNow; //For pacekeeping I believe
  float time_step  = 1.0f / RATE; //Time step for integration

  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {
    lowGimu.readAccel();
    lowGimu.readGyro();
    lowGimu.readMag();
    // Get accelerometer readings in g
    float ax = lowGimu.calcAccel(lowGimu.ax);
    float ay = lowGimu.calcAccel(lowGimu.ay);
    float az = lowGimu.calcAccel(lowGimu.az);

    // Get gyro readings in degrees per second
    float gx = lowGimu.calcGyro(lowGimu.gx);
    float gy = lowGimu.calcGyro(lowGimu.gy);
    float gz = lowGimu.calcGyro(lowGimu.gz);

    //Get mag readings
    float mx = lowGimu.calcMag(lowGimu.mx);
    float my = lowGimu.calcMag(lowGimu.my);
    float mz = lowGimu.calcMag(lowGimu.mz);


    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
    // print the yaw, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    yaw = filter.getYaw();
    q0 = filter.getq0();
    q1 = filter.getq1();
    q2 = filter.getq2();
    q3 = filter.getq3();
    Serial.print("Orientation: ");
    Serial.print(yaw);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);

    // Getting the co-ordinate transformation matrix for body to inertial frame
    float Body_to_Inertial[3][3];
    body_to_inertial(Body_to_Inertial, q0, q1, q2, q3);

    //Convert IMU readings to inertial accelerations
    float aix, aiy, aiz;
    specf_to_ai(&aix, &aiy, &aiz, ax, ay, az, Body_to_Inertial);


    //Integrate acceleration to get velocities (Groves, pg. 171)
    vx += aix*time_step;
    vy += aiy*time_step;
    vz += aiz*time_step;

    //Position update (Groves, pg. 172)
    x += vx*time_step -  (float) (aix * time_step * time_step)/ (float) 2;
    y += vy*time_step -  (float) (aiy * time_step * time_step)/ (float) 2;
    z += vz*time_step -  (float) (aiz * time_step * time_step)/ (float) 2;
    

    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;

  }

}