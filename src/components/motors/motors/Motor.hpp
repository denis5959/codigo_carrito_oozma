#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <Arduino.h>

class Motor {
private:
    int pin_in1, pin_in2, pin_en;
    bool invertido;

public:
    Motor(int in1, int in2, int en, bool inv);
    void begin();
    void set(int velocidad);
    void detener();
};

#endif