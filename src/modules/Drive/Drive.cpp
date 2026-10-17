#include "Drive.hpp"
#include "constants/driveConfig.h"

Drive::Drive(Motor delantero_der, Motor delantero_izq, Motor trasero_der, Motor trasero_izq)
    : del_der(delantero_der), del_izq(delantero_izq), tra_der(trasero_der), tra_izq(trasero_izq) {}

void Drive::begin() {
    del_der.begin();
    del_izq.begin();
    tra_der.begin();
    tra_izq.begin();
}

void Drive::setLadoIzquierdo(int velocidad) {
    del_izq.set(velocidad);
    tra_izq.set(velocidad);
}

void Drive::setLadoDerecho(int velocidad) {
    int velocidad_ajustada = (int)(velocidad * MULTIPLICADOR_LADO_DERECHO);
    del_der.set(velocidad_ajustada);
    tra_der.set(velocidad_ajustada);
}

void Drive::carritoDrive(int vel_izq, int vel_der) {
    setLadoIzquierdo(vel_izq);
    setLadoDerecho(vel_der);
}

void Drive::avanzar(int velocidad) {
    carritoDrive(velocidad, velocidad);
}

void Drive::retroceder(int velocidad) {
    carritoDrive(-velocidad, -velocidad);
}

void Drive::girarSobreEje(int velocidad) {
#if defined(PISTA_A)
    del_der.set(velocidad);
    tra_der.set(-velocidad);
    del_izq.set(-velocidad);
    tra_izq.set(velocidad);
#else
    const int velocidad_derecha = (int)(velocidad * MULTIPLICADOR_LADO_DERECHO);

    del_der.set(velocidad_derecha);
    tra_der.set(-velocidad_derecha);
    del_izq.set(-velocidad);
    tra_izq.set(velocidad);
#endif
}

void Drive::detener() {
    carritoDrive(0, 0);
}

void Drive::aplicar(float fl, float fr, float rl, float rr) {
    fr *= MULTIPLICADOR_LADO_DERECHO;
    rr *= MULTIPLICADOR_LADO_DERECHO;

    float mayor = max(max(abs(fl), abs(fr)), max(abs(rl), abs(rr)));
    float escala = (mayor > 255.0) ? (255.0 / mayor) : 1.0;

    del_izq.set((int)(fl * escala));
    del_der.set((int)(fr * escala));
    tra_izq.set((int)(rl * escala));
    tra_der.set((int)(rr * escala));
}

void Drive::deslizar(int velocidad) {
    mover(0, velocidad, 0);
}

void Drive::mover(int vx, int vy, int giro) {
    if (!ES_MECANUM && vy != 0) {
        detener();
        return;
    }

    float lateral = (float)vy * STRAFE_SIGNO;

    float fl = vx + lateral - giro;
    float fr = vx - lateral + giro;
    float rl = vx - lateral - giro;
    float rr = vx + lateral + giro;

    aplicar(fl, fr, rl, rr);
}