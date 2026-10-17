#include "MPU6050.hpp"
#include "constants/driveConfig.h"
#include "constants/i2cConfig.h"

MPU6050Component::MPU6050Component(byte addr) {
    address = addr;
    angulo_x = 0.0; angulo_y = 0.0; angulo_z = 0.0;
    tiempo_anterior = 0;
    velocidad_z_actual = 0.0;
    offset_gx = OFFSET_GX;
    offset_gy = OFFSET_GY;
    offset_gz = OFFSET_GZ;
}

void MPU6050Component::escribirRegistro(byte registro, byte valor) {
    seleccionarCanalI2C(MPU_I2C_CHANNEL);
    Wire.beginTransmission(address);
    Wire.write(registro);
    Wire.write(valor);
    Wire.endTransmission(true);
}

void MPU6050Component::begin() {
    seleccionarCanalI2C(MPU_I2C_CHANNEL);
    escribirRegistro(0x6B, 0x00);
    escribirRegistro(0x1B, 0x00);
    delay(100);

    if (CALIBRAR_GIRO_AL_INICIO) {
        int32_t suma_gx = 0;
        int32_t suma_gy = 0;
        int32_t suma_gz = 0;
        int muestras_validas = 0;

        for (int i = 0; i < MUESTRAS_CALIBRACION_GIRO; i++) {
            seleccionarCanalI2C(MPU_I2C_CHANNEL);
            Wire.beginTransmission(address);
            Wire.write(0x43);
            if (Wire.endTransmission(false) == 0 &&
                Wire.requestFrom(address, (byte)6, (byte)true) == 6) {
                int16_t gx = (int16_t)((Wire.read() << 8) | Wire.read());
                int16_t gy = (int16_t)((Wire.read() << 8) | Wire.read());
                int16_t gz = (int16_t)((Wire.read() << 8) | Wire.read());
                suma_gx += gx;
                suma_gy += gy;
                suma_gz += gz;
                muestras_validas++;
            }
            delay(2);
        }

        if (muestras_validas == MUESTRAS_CALIBRACION_GIRO) {
            offset_gx = (float)suma_gx / muestras_validas;
            offset_gy = (float)suma_gy / muestras_validas;
            offset_gz = (float)suma_gz / muestras_validas;
            Serial.println(F("Offsets MPU calibrados. Mantener robot inmovil al iniciar."));
        } else {
            Serial.println(F("No se pudo completar la calibracion MPU; se conservan los offsets configurados."));
        }
    }

    tiempo_anterior = micros();
}

float MPU6050Component::aplicarZonaMuerta(float velocidad_dps) {
    if (velocidad_dps > -ZONA_MUERTA_DPS && velocidad_dps < ZONA_MUERTA_DPS) {
        return 0.0;
    }
    return velocidad_dps;
}

bool MPU6050Component::actualizar() {
    seleccionarCanalI2C(MPU_I2C_CHANNEL);
    Wire.beginTransmission(address);
    Wire.write(0x43);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(address, (byte)6, (byte)true) != 6) return false;

    int16_t gx_raw = (Wire.read() << 8) | Wire.read();
    int16_t gy_raw = (Wire.read() << 8) | Wire.read();
    int16_t gz_raw = (Wire.read() << 8) | Wire.read();

    float gx_dps = (gx_raw - offset_gx) / SENSIBILIDAD_GIRO;
    float gy_dps = (gy_raw - offset_gy) / SENSIBILIDAD_GIRO;
    float gz_dps = (gz_raw - offset_gz) / SENSIBILIDAD_GIRO;

    velocidad_z_actual = aplicarZonaMuerta(gz_dps);

    unsigned long tiempo_actual = micros();
    float dt = (tiempo_actual - tiempo_anterior) / 1000000.0;
    tiempo_anterior = tiempo_actual;

    angulo_x += aplicarZonaMuerta(gx_dps) * dt;
    angulo_y += aplicarZonaMuerta(gy_dps) * dt;
    angulo_z += velocidad_z_actual * dt;

    return true;
}

float MPU6050Component::getAnguloZ() const { return angulo_z; }
float MPU6050Component::getVelocidadZ() const { return velocidad_z_actual; }
void MPU6050Component::resetAnguloZ(float val) { angulo_z = val; }