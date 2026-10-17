#ifndef DRIVE_HPP
#define DRIVE_HPP

#include <Arduino.h>
#include "components/motors/motors/Motor.hpp"

class Drive {
private:
    Motor del_der;
    Motor del_izq;
    Motor tra_der;
    Motor tra_izq;

    void aplicar(float fl, float fr, float rl, float rr);

public:
    Drive(Motor delantero_der, Motor delantero_izq, Motor trasero_der, Motor trasero_izq);
    void begin();

    void setLadoIzquierdo(int velocidad);
    void setLadoDerecho(int velocidad);
    void carritoDrive(int vel_izq, int vel_der);
    void avanzar(int velocidad);
    void retroceder(int velocidad);
    void girarSobreEje(int velocidad);
    void detener();

    void deslizar(int velocidad);
    void mover(int vx, int vy, int giro);
};

#endif