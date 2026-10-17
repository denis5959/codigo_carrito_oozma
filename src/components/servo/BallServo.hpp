#ifndef BALL_SERVO_HPP
#define BALL_SERVO_HPP

#include <Arduino.h>
#include <Servo.h>

class BallServo {
private:
    Servo servo;
    uint8_t pin;
    uint8_t angulo_abierto;
    uint8_t angulo_contenido;
    bool conectado;

public:
    BallServo();
    bool begin(uint8_t servo_pin, uint8_t open_angle, uint8_t contain_angle);
    void abrir();
    void contener();
};

#endif
