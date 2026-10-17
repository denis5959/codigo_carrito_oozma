#ifndef PISTA2_HPP
#define PISTA2_HPP

#include <Arduino.h>
#include "components/color_sensor/ColorSensor.hpp"
#include "components/servo/BallServo.hpp"
#include "components/ultrasonic/UltrasonicSensor.hpp"

class RobotManager;

class Pista2 {
private:
    enum Estado : uint8_t {
        AVANCE_INICIAL,
        IDENTIFICAR_SECCION,
        P1_AVANZAR,
        P1_EVALUAR_DERECHA,
        P1_MEDIR_RETROCESO,
        P1_RETROCEDER_PELOTA,
        P1_ELEGIR_SALIDA,
        P1_SALIR,
        P2_AVANZAR,
        P2_MEDIR_LATERAL,
        P2_DESLIZAR,
        P3_ESPERAR_FICHA,
        P3_PAUSA_FICHA,
        P3_SEGUIR_FICHA,
        GIRO_GYRO,
        TERMINADO,
        ERROR
    };

    Estado estado;
    Estado estado_despues_giro;
    UltrasonicSensor sensor_frente;
    UltrasonicSensor sensor_izquierdo;
    UltrasonicSensor sensor_derecho;
    ColorSensorComponent sensor_color_2;
    BallServo compuerta;

    ColorDetectado ficha_actual;
    ColorDetectado ficha_pendiente;
    float distancia_referencia_frente;
    float objetivo_retroceso;
    int8_t direccion_lateral;
    int16_t grados_salida;
    uint8_t lecturas_retroceso;
    uint8_t lecturas_fuera_blanco;
    bool ficha_actual_abandonada;

    unsigned long marca_tiempo;
    unsigned long tiempo_ultima_medicion;
    unsigned long tiempo_movimiento_ficha;
    unsigned long tiempo_actualizar_color_secundario;

    ColorDetectado leerColorPrimario(const RobotManager &robot) const;
    ColorDetectado leerColorSecundario() const;
    float medirDistancia(UltrasonicSensor &sensor);
    void avanzarConRumbo(RobotManager &robot, int velocidad);
    void iniciarGiro(RobotManager &robot, float grados,
                     Estado siguiente);
    void completarGiro(RobotManager &robot);
    bool leerParColores(const RobotManager &robot,
                        ColorDetectado &primario,
                        ColorDetectado &secundario);
    void seleccionarSalida();
    bool esFichaDireccion(ColorDetectado color) const;
    void iniciarFicha(ColorDetectado color);
    void fallar(RobotManager &robot, const __FlashStringHelper *mensaje);

public:
    Pista2();
    void begin(RobotManager &robot);
    void periodic(RobotManager &robot);
};

#endif
