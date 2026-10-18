#ifndef PISTA2_HPP
#define PISTA2_HPP

#include <Arduino.h>
#include "components/color_sensor/ColorSensor.hpp"
#include "components/ultrasonic/UltrasonicSensor.hpp"

class RobotManager;

class Pista2 {
private:
    enum Estado : uint8_t {
        AVANCE_INICIAL,
        IDENTIFICAR_SECCION,
        P1_TRAMO_1,
        P1_TRAMO_2,
        P1_TRAMO_CENTRO,
        P1_BUSCAR_CHECKPOINT_ROJO,
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

    ColorDetectado ficha_actual;
    ColorDetectado ficha_pendiente;
    float objetivo_distancia_centro;
    int8_t direccion_lateral;
    uint8_t lecturas_pared;
    uint8_t lecturas_centro;
    uint8_t lecturas_checkpoint_rojo;
    uint8_t lecturas_fuera_blanco;
    uint8_t lecturas_giro_estable;

    unsigned long marca_tiempo;
    unsigned long tiempo_ultima_medicion;
    unsigned long tiempo_movimiento_ficha;
    unsigned long tiempo_actualizar_color_secundario;
    unsigned long inicio_tramo_pista_a;
    unsigned long tiempo_ultimo_reporte_color;
    unsigned long tiempo_anterior_pantalla;

    ColorDetectado leerColorPrimario(const RobotManager &robot) const;
    ColorDetectado leerColorSecundario() const;
    const __FlashStringHelper *nombrePartePista() const;
    float medirDistancia(UltrasonicSensor &sensor);
    void avanzarConRumbo(RobotManager &robot, int velocidad);
    void iniciarGiro(RobotManager &robot, float grados,
                     Estado siguiente);
    void completarGiro(RobotManager &robot);
    bool leerParColores(const RobotManager &robot,
                        ColorDetectado &primario,
                        ColorDetectado &secundario);
    bool esFichaDireccion(ColorDetectado color) const;
    void iniciarFicha(ColorDetectado color);
    void fallar(RobotManager &robot, const __FlashStringHelper *mensaje);

public:
    Pista2();
    void begin(RobotManager &robot);
    void periodic(RobotManager &robot);
};

#endif
