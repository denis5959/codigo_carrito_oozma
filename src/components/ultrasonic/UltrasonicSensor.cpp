#include "UltrasonicSensor.hpp"

UltrasonicSensor::UltrasonicSensor(uint8_t trigger, uint8_t echo)
    : trigger_pin(trigger), echo_pin(echo) {}

void UltrasonicSensor::begin() {
    pinMode(trigger_pin, OUTPUT);
    pinMode(echo_pin, INPUT);
    digitalWrite(trigger_pin, LOW);
}

float UltrasonicSensor::medirCm(unsigned long timeout_us) const {
    digitalWrite(trigger_pin, LOW);
    delayMicroseconds(3);
    digitalWrite(trigger_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger_pin, LOW);

    unsigned long duracion = pulseIn(echo_pin, HIGH, timeout_us);
    if (duracion == 0) return 0.0;
    return duracion * 0.0343f / 2.0f;
}
