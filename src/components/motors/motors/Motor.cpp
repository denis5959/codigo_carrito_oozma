#include "Motor.hpp"

Motor::Motor(int in1, int in2, int en, bool inv)
    : pin_in1(in1), pin_in2(in2), pin_en(en), invertido(inv) {}

void Motor::begin() {
    pinMode(pin_in1, OUTPUT);
    pinMode(pin_in2, OUTPUT);
    pinMode(pin_en, OUTPUT);
    detener();
}

void Motor::set(int velocidad) {
    if (invertido) {
        velocidad = -velocidad;
    }

    velocidad = constrain(velocidad, -255, 255);

    if (velocidad > 0) {
        digitalWrite(pin_in1, HIGH);
        digitalWrite(pin_in2, LOW);
    } else if (velocidad < 0) {
        digitalWrite(pin_in1, LOW);
        digitalWrite(pin_in2, HIGH);
    } else {
        digitalWrite(pin_in1, LOW);
        digitalWrite(pin_in2, LOW);
    }

    analogWrite(pin_en, abs(velocidad));
}

void Motor::detener() {
    set(0);
}