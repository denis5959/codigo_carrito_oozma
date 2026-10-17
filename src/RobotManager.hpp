#ifndef ROBOT_MANAGER_HPP
#define ROBOT_MANAGER_HPP

#include <Arduino.h>
#include <Wire.h>
#include "modules/Drive/Drive.hpp"
#include "components/imu/MPU6050.hpp"
#include "components/color_sensor/ColorSensor.hpp"
#include "components/ir_sensor/IRSensor.hpp"
#include "components/display/OLEDDisplay.hpp"
#include "components/motors/controller/PIDController.hpp"
#include "constants/pinsConfig.h"
#include "constants/driveConfig.h"
#include "constants/colorConfig.h"

class RobotManager {
public:
    Drive drive;
    MPU6050Component mpu;
    ColorSensorComponent colorSensor;
    OLEDDisplayComponent oled;
    IRSensorComponent irPiso1;
    IRSensorComponent irPiso2;

private:
    PIDController pid;

    float angulo_objetivo;
    ColorDetectado color_anterior;
    bool color_cambio;

    unsigned long tiempo_anterior_oled;
    unsigned long tiempo_anterior_pid;
    unsigned long tiempo_anterior_color;

    float normalizar360(float angulo) const;
    float calcularErrorAngular(float objetivo, float actual) const;

public:
    RobotManager();
    void begin();

    bool actualizar();
    void terminarCiclo();

    void fijarRumbo(float grados);
    void girarRelativo(float grados);
    void controlarRumbo();
    float correccionRumbo(float limite);
    bool enRumbo() const;
    float getRumbo() const;
    float getObjetivo() const;
    float getError() const;
    float getVelocidadZ() const;

    ColorDetectado getColor() const;
    bool colorCambio() const;
};

#endif