#include <Arduino.h>
#include <SPI.h>

#include "KX134-1211.h"
#include "kx134_1211_Registers.h"

// #define DEBUG

//constructor
KX134::KX134()
{
    //Do any necessary setup
    x_accel = 0;
    y_accel = 0;
    z_accel = 0;
    init();
}

//Returns the raw binary data
int16_t KX134::get_x_accel_raw() {
    return x_accel;
}
int16_t KX134::get_y_accel_raw() {
    return y_accel;
}
int16_t KX134::get_z_accel_raw() {
    return z_accel;
}

//  Range from -64 to 64 Gs. 511.98 obtained by dividing 32767 (max decimal) by 64 (mag gs)
float KX134::get_x_gforce() {
    int16_t decimal = get_x_accel_raw();
    float gForce = (float)decimal / 512;
    return gForce;
}
float KX134::get_y_gforce() {
    int16_t decimal = get_y_accel_raw();
    float gForce = (float)decimal / 512;
    return gForce;
}
float KX134::get_z_gforce() {
    int16_t decimal = get_z_accel_raw();
    float gForce = (float)decimal / 512;
    return gForce;
}

//  Getters for acceleratoin (m/s^2). Derived from g force
//  Multiply the gForce by 9.8m/s^2 to get acceleration
float KX134::get_x_accel() {
    float gForce = get_x_gforce();
    return gForce * 9.8;
}
float KX134::get_y_accel() {
    float gForce = get_y_gforce();
    return gForce * 9.8;
}
float KX134::get_z_accel() {
    float gForce = get_z_gforce();
    return gForce * 9.8;
}

void KX134::init()
{
    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV32);

    pinMode(KX134_CS_PIN, OUTPUT);

    // Set PC1 bit of CNTL1 register to enable measurements
    digitalWrite(KX134_CS_PIN, LOW);   // select chip
    SPI.transfer(CNTL1);
    //TODO switch to 88 for +-16 Gs
    SPI.transfer(0x98);
    digitalWrite(KX134_CS_PIN, HIGH);   // de-select chip

}

void KX134::update_data()
{
    uint8_t data[6];

    digitalWrite(KX134_CS_PIN, LOW); // select chip
    delayMicroseconds(1);

    SPI.transfer(0x80 | XOUT_L);

    for (int i = 0; i < 6; ++i) {
        data[i] = SPI.transfer(0x00);   // 0b11111111
    }

    digitalWrite(KX134_CS_PIN, HIGH); // de-select chip

    x_accel = (data[1] << 8) | data[0];
    y_accel = (data[3] << 8) | data[2];
    z_accel = (data[5] << 8) | data[4];

#ifdef DEBUG
    Serial.print(x_accel);
    Serial.print(",\t");
    Serial.print(y_accel);
    Serial.print(",\t");
    Serial.println(z_accel);
#endif

}