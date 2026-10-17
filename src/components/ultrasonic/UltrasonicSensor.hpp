#ifndef ULTRASONIC_SENSOR_HPP
#define ULTRASONIC_SENSOR_HPP

#include <Arduino.h>

class UltrasonicSensor {
private:
    uint8_t trigger_pin;
    uint8_t echo_pin;

public:
    UltrasonicSensor(uint8_t trigger, uint8_t echo);
    void begin();
    float medirCm(unsigned long timeout_us = 24000UL) const;
};

#endif
