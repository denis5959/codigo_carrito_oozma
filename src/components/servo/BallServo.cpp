#include "BallServo.hpp"

BallServo::BallServo()
    : pin(0), angulo_abierto(20), angulo_contenido(90),
      conectado(false) {}

bool BallServo::begin(uint8_t servo_pin, uint8_t open_angle,
                      uint8_t contain_angle) {
    if (open_angle > 180 || contain_angle > 180) return false;

    pin = servo_pin;
    angulo_abierto = open_angle;
    angulo_contenido = contain_angle;
    conectado = servo.attach(pin) != 0;
    if (conectado) abrir();
    return conectado;
}

void BallServo::abrir() {
    if (conectado) servo.write(angulo_abierto);
}

void BallServo::contener() {
    if (conectado) servo.write(angulo_contenido);
}
