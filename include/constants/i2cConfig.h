#ifndef I2C_CONFIG_H
#define I2C_CONFIG_H

#include <Arduino.h>
#include <Wire.h>

const byte I2C_MULTIPLEXER_ADDRESS = 0x70;
const byte OLED_I2C_CHANNEL = 4;
const byte COLOR_SENSOR_I2C_CHANNEL = 2;
const byte MPU_I2C_CHANNEL = 5;

inline void seleccionarCanalI2C(byte canal) {
    if (canal > 7) return;
    Wire.beginTransmission(I2C_MULTIPLEXER_ADDRESS);
    Wire.write((byte)(1U << canal));
    Wire.endTransmission();
}

#endif
