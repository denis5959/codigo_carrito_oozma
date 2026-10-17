#include "IRSensor.hpp"

IRSensorComponent::IRSensorComponent(int pin_sensor, bool activo_en_alto)
    : pin(pin_sensor), detecta_en_alto(activo_en_alto) {}

void IRSensorComponent::begin() {
    pinMode(pin, INPUT);
}

bool IRSensorComponent::detecta() const {
    bool nivel_alto = (digitalRead(pin) == HIGH);
    return detecta_en_alto ? nivel_alto : !nivel_alto;
}