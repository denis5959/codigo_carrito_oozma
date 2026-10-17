#ifndef PISTA1_HPP
#define PISTA1_HPP

#include <Arduino.h>
#include "components/color_sensor/ColorSensor.hpp"
#include "components/ultrasonic/UltrasonicSensor.hpp"

class RobotManager;

class Pista1 {
private:
    static const uint8_t MAPA_LADO = 11;
    static const uint8_t MAX_CELDAS = 25;
    static const uint8_t PROFUNDIDAD_MAXIMA = 25;

    enum Estado : uint8_t {
        ANALIZANDO,
        ESPERA_ANALISIS,
        GIRANDO,
        AVANZANDO,
        RETROCEDIENDO_OBSTACULO,
        PAUSA_COLOR,
        PAUSA_META_FINAL,
        META,
        SIN_RUTA
    };

    enum Fase : uint8_t {
        EXPLORANDO,
        YENDO_A_META,
        REGRESANDO_INICIO
    };

    enum TipoMovimiento : uint8_t {
        AVANCE_EXPLORACION,
        RETORNO_EXPLORACION,
        RUTA_A_META,
        RUTA_A_INICIO
    };

    Estado estado;
    Estado estado_antes_pausa;
    Fase fase;
    TipoMovimiento tipo_movimiento;
    UltrasonicSensor sensor_frente;
    UltrasonicSensor sensor_izquierdo;
    UltrasonicSensor sensor_derecho;

    uint8_t celdas_visitadas[MAPA_LADO][MAPA_LADO];
    uint8_t salidas_abiertas[MAPA_LADO][MAPA_LADO];
    uint8_t salidas_conocidas[MAPA_LADO][MAPA_LADO];
    int8_t historial_x[PROFUNDIDAD_MAXIMA];
    int8_t historial_y[PROFUNDIDAD_MAXIMA];
    uint8_t historial_direccion[PROFUNDIDAD_MAXIMA];
    uint8_t ruta_meta[MAX_CELDAS];

    int8_t x;
    int8_t y;
    int8_t inicio_x;
    int8_t inicio_y;
    int8_t meta_x;
    int8_t meta_y;
    int8_t destino_x;
    int8_t destino_y;
    uint8_t rumbo_celda;
    uint8_t rumbo_destino;
    uint8_t profundidad;
    uint8_t celdas_unicas;
    uint8_t longitud_ruta_meta;
    uint8_t indice_ruta;
    uint16_t numero_casilla;
    uint16_t colores_detectados;
    bool meta_encontrada;

    unsigned long inicio_movimiento;
    unsigned long inicio_giro;
    unsigned long tiempo_limite_giro;
    unsigned long inicio_pausa;
    unsigned long inicio_espera_analisis;
    unsigned long inicio_retroceso_obstaculo;
    unsigned long duracion_retroceso_obstaculo;
    unsigned long tiempo_ultima_lectura;
    bool regreso_obstaculo_pendiente;

    void analizarCasilla(RobotManager &robot);
    void iniciarMovimiento(RobotManager &robot, uint8_t direccion, TipoMovimiento tipo);
    void iniciarRetornoPorObstaculo(RobotManager &robot, unsigned long ahora);
    void iniciarAvance(RobotManager &robot);
    void completarMovimiento(RobotManager &robot);
    void procesarColor(RobotManager &robot, ColorDetectado color);
    void registrarSalida(int8_t cx, int8_t cy, uint8_t direccion, bool abierta);
    bool calcularRutaMeta();
    void iniciarSiguienteTramoRuta(RobotManager &robot);
    bool dentroDelMapa(int8_t cx, int8_t cy) const;
    bool salidaAbierta(float distancia, float umbral) const;

public:
    Pista1();
    void begin(RobotManager &robot);
    void periodic(RobotManager &robot);
};

#endif
