#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <Arduino.h>

class PIDController {
private:
    float kp, ki, kd;
    float error_anterior;
    float integral_error;

public:
    PIDController(float p, float i, float d);
    void reset();
    void reposo(float error);
    float calcular(float error, float dt);
};

#endif