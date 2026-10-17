#ifndef MPU6050_HPP
#define MPU6050_HPP

#include <Arduino.h>
#include <Wire.h>

class MPU6050Component {
private:
    byte address;
    unsigned long tiempo_anterior;
    float angulo_x, angulo_y, angulo_z;
    float velocidad_z_actual;
    float offset_gx, offset_gy, offset_gz;

    void escribirRegistro(byte registro, byte valor);
    float aplicarZonaMuerta(float velocidad_dps);

public:
    MPU6050Component(byte addr);
    void begin();
    bool actualizar();
    float getAnguloZ() const;
    float getVelocidadZ() const;
    void resetAnguloZ(float val = 0.0);
};

#endif