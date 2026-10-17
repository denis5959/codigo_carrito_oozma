#include "PIDController.hpp"

PIDController::PIDController(float p, float i, float d) : kp(p), ki(i), kd(d) {
    reset();
}

void PIDController::reset() {
    error_anterior = 0.0;
    integral_error = 0.0;
}

void PIDController::reposo(float error) {
    integral_error = 0.0;
    error_anterior = error;
}

float PIDController::calcular(float error, float dt) {
    if (dt <= 0.0) dt = 0.001;

    if (abs(error) < 25.0) {
        integral_error += error * dt;
    } else {
        integral_error = 0.0;
    }
    integral_error = constrain(integral_error, -80.0, 80.0);

    float derivada = (error - error_anterior) / dt;
    error_anterior = error;

    return (kp * error) + (ki * integral_error) + (kd * derivada);
}