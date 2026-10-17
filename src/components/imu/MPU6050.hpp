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
    float aceleracion_base_x, aceleracion_base_y, aceleracion_base_z;
    float aceleracion_filtrada_x, aceleracion_filtrada_y, aceleracion_filtrada_z;
    float inclinacion_grados;
    bool referencia_acelerometro_valida;

    void escribirRegistro(byte registro, byte valor);
    float aplicarZonaMuerta(float velocidad_dps);
    bool leerAceleracionRaw(int16_t &ax, int16_t &ay, int16_t &az);

public:
    MPU6050Component(byte addr);
    void begin();
    bool actualizar();
    float getAnguloZ() const;
    float getVelocidadZ() const;
    bool getInclinacionGrados(float &grados) const;
    void resetAnguloZ(float val = 0.0);
};

#endif