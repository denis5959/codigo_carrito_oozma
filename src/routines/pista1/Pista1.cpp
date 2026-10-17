#include "Pista1.hpp"
#include "RobotManager.hpp"
#include "constants/driveConfig.h"
#include "constants/pinsConfig.h"

namespace {
const float DISTANCIA_SEGURIDAD_CM = 12.0;
const float UMBRAL_PARED_FRENTE_CM = 20.0;
const float UMBRAL_PARED_LADO_CM = 12.0;
const int VELOCIDAD_AVANCE = 42;
const int CORRECCION_RUMBO_MAXIMA = 7;
const unsigned long TIEMPO_CASILLA_MS = 1750;
const unsigned long TIEMPO_MAXIMO_CASILLA_MS = 6000;
const unsigned long TIEMPO_FRENADO_MS = 300;
const unsigned long TIEMPO_PAUSA_COLOR_MS = 5000;
const unsigned long TIEMPO_PAUSA_META_MS = 5000;
const unsigned long TIEMPO_ESTABILIZAR_CASILLA_MS = 800;
const unsigned long INTERVALO_ULTRASONICO_MS = 100;
const uint8_t DIRECCION_DESCONOCIDA = 0xFF;

uint8_t mascaraDireccion(uint8_t direccion) {
    return (uint8_t)(1U << (direccion & 3U));
}

int8_t cambioX(uint8_t direccion) {
    if (direccion == 1) return -1;
    if (direccion == 3) return 1;
    return 0;
}

int8_t cambioY(uint8_t direccion) {
    if (direccion == 0) return 1;
    if (direccion == 2) return -1;
    return 0;
}

float mediana3(float a, float b, float c) {
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
}

int velocidadAvance(unsigned long transcurrido, unsigned long duracion,
                    bool frenarSuave) {
    if (duracion == 0 || transcurrido >= duracion) return 0;
    if (!frenarSuave) return VELOCIDAD_AVANCE;

    const unsigned long tiempo_frenado =
        duracion < TIEMPO_FRENADO_MS ? duracion : TIEMPO_FRENADO_MS;
    const unsigned long inicio_frenado = duracion - tiempo_frenado;
    if (transcurrido < inicio_frenado) return VELOCIDAD_AVANCE;

    const unsigned long restante = duracion - transcurrido;
    return (int)((VELOCIDAD_AVANCE * restante) / tiempo_frenado);
}

void avanzarConCorreccion(RobotManager &robot, unsigned long transcurrido,
                          unsigned long duracion, bool frenarSuave) {
    const int velocidad =
        velocidadAvance(transcurrido, duracion, frenarSuave);
    if (velocidad == 0) {
        robot.drive.detener();
        return;
    }

    const float proporcion = (float)velocidad / VELOCIDAD_AVANCE;
    const int giro = (int)(robot.correccionRumbo(CORRECCION_RUMBO_MAXIMA) *
                           proporcion);
    robot.drive.mover(velocidad, 0, giro);
}

float medirFiltrado(UltrasonicSensor &sensor) {
    float a = sensor.medirCm(18000UL);
    delay(25);
    float b = sensor.medirCm(18000UL);
    delay(25);
    float c = sensor.medirCm(18000UL);

    if (a == 0.0) a = 999.0;
    if (b == 0.0) b = 999.0;
    if (c == 0.0) c = 999.0;
    return mediana3(a, b, c);
}

bool esColorMarcador(ColorDetectado color) {
    return color == ColorDetectado::CIAN ||
           color == ColorDetectado::MAGENTA ||
           color == ColorDetectado::AMARILLO ||
           color == ColorDetectado::NARANJA;
}
}

Pista1::Pista1()
    : estado(ANALIZANDO), estado_antes_pausa(ANALIZANDO),
      fase(EXPLORANDO), tipo_movimiento(AVANCE_EXPLORACION),
      sensor_frente(TRIG_ADELANTE, ECHO_ADELANTE),
      sensor_izquierdo(TRIG_IZQ, ECHO_IZQ),
      sensor_derecho(TRIG_DER, ECHO_DER),
      x(5), y(5), inicio_x(5), inicio_y(5), meta_x(0), meta_y(0),
      destino_x(5), destino_y(5),
      rumbo_celda(0), rumbo_destino(0), profundidad(0), celdas_unicas(1),
      longitud_ruta_meta(0), indice_ruta(0), numero_casilla(1),
      colores_detectados(0), meta_encontrada(false),
      inicio_movimiento(0), inicio_giro(0), tiempo_limite_giro(0),
      inicio_pausa(0), inicio_espera_analisis(0),
      inicio_retroceso_obstaculo(0), duracion_retroceso_obstaculo(0),
      tiempo_ultima_lectura(0),
      regreso_obstaculo_pendiente(false) {
    memset(celdas_visitadas, 0, sizeof(celdas_visitadas));
    memset(salidas_abiertas, 0, sizeof(salidas_abiertas));
    memset(salidas_conocidas, 0, sizeof(salidas_conocidas));
    memset(historial_x, 0, sizeof(historial_x));
    memset(historial_y, 0, sizeof(historial_y));
    memset(historial_direccion, 0, sizeof(historial_direccion));
    memset(ruta_meta, DIRECCION_DESCONOCIDA, sizeof(ruta_meta));
}

void Pista1::begin(RobotManager &robot) {
    sensor_frente.begin();
    sensor_izquierdo.begin();
    sensor_derecho.begin();

    x = MAPA_LADO / 2;
    y = MAPA_LADO / 2;
    inicio_x = x;
    inicio_y = y;
    rumbo_celda = 0;
    profundidad = 0;
    celdas_unicas = 1;
    numero_casilla = 1;
    celdas_visitadas[y][x] = 1;
    robot.oled.setCasilla(numero_casilla);
    robot.fijarRumbo(robot.getRumbo());

    Serial.println(F("Exploracion de laberinto inicializada."));
    estado = ANALIZANDO;
}

bool Pista1::dentroDelMapa(int8_t cx, int8_t cy) const {
    return cx >= 0 && cy >= 0 &&
           cx < (int8_t)MAPA_LADO && cy < (int8_t)MAPA_LADO;
}

bool Pista1::salidaAbierta(float distancia, float umbral) const {
    return distancia >= umbral;
}

void Pista1::registrarSalida(int8_t cx, int8_t cy, uint8_t direccion, bool abierta) {
    if (!dentroDelMapa(cx, cy)) return;

    uint8_t bit = mascaraDireccion(direccion);
    salidas_conocidas[cy][cx] |= bit;
    if (abierta) salidas_abiertas[cy][cx] |= bit;
    else salidas_abiertas[cy][cx] &= (uint8_t)~bit;

    int8_t nx = cx + cambioX(direccion);
    int8_t ny = cy + cambioY(direccion);
    if (!dentroDelMapa(nx, ny)) return;

    uint8_t opuesta = (uint8_t)((direccion + 2U) & 3U);
    uint8_t bit_opuesto = mascaraDireccion(opuesta);
    salidas_conocidas[ny][nx] |= bit_opuesto;
    if (abierta) salidas_abiertas[ny][nx] |= bit_opuesto;
    else salidas_abiertas[ny][nx] &= (uint8_t)~bit_opuesto;
}

void Pista1::analizarCasilla(RobotManager &robot) {
    robot.drive.detener();
    delay(50);

    float frente = medirFiltrado(sensor_frente);
    delay(30);
    float izquierda = medirFiltrado(sensor_izquierdo);
    delay(30);
    float derecha = medirFiltrado(sensor_derecho);

    Serial.print(F("Celda "));
    Serial.print(numero_casilla);
    Serial.print(F(" ("));
    Serial.print(x);
    Serial.print(',');
    Serial.print(y);
    Serial.print(F(") F/I/D cm: "));
    Serial.print(frente, 0);
    Serial.print('/');
    Serial.print(izquierda, 0);
    Serial.print('/');
    Serial.println(derecha, 0);

    registrarSalida(x, y, rumbo_celda,
                    salidaAbierta(frente, UMBRAL_PARED_FRENTE_CM));
    registrarSalida(x, y, (uint8_t)((rumbo_celda + 1U) & 3U),
                    salidaAbierta(izquierda, UMBRAL_PARED_LADO_CM));
    registrarSalida(x, y, (uint8_t)((rumbo_celda + 3U) & 3U),
                    salidaAbierta(derecha, UMBRAL_PARED_LADO_CM));

    const uint8_t prioridad[3] = {
        rumbo_celda,
        (uint8_t)((rumbo_celda + 1U) & 3U),
        (uint8_t)((rumbo_celda + 3U) & 3U)
    };

    for (uint8_t i = 0; i < 3; i++) {
        uint8_t direccion = prioridad[i];
        if ((salidas_abiertas[y][x] & mascaraDireccion(direccion)) == 0)
            continue;

        int8_t nx = x + cambioX(direccion);
        int8_t ny = y + cambioY(direccion);
        if (!dentroDelMapa(nx, ny) || celdas_visitadas[ny][nx]) continue;

        if (celdas_unicas >= MAX_CELDAS) {
            Serial.println(F("Limite de celdas alcanzado."));
            estado = SIN_RUTA;
            return;
        }

        iniciarMovimiento(robot, direccion, AVANCE_EXPLORACION);
        return;
    }

    if (profundidad > 0) {
        uint8_t direccion = (uint8_t)((historial_direccion[profundidad - 1] + 2U) & 3U);
        iniciarMovimiento(robot, direccion, RETORNO_EXPLORACION);
        return;
    }

    if (!meta_encontrada || !calcularRutaMeta()) {
        robot.drive.detener();
        estado = SIN_RUTA;
        Serial.println(F("Exploracion completa; no se encontro ruta a la meta roja."));
        return;
    }

    fase = YENDO_A_META;
    indice_ruta = 0;
    Serial.print(F("Exploracion completa: "));
    Serial.print(celdas_unicas);
    Serial.println(F(" celdas. Ruta final calculada."));
    iniciarSiguienteTramoRuta(robot);
}

bool Pista1::calcularRutaMeta() {
    if (!dentroDelMapa(meta_x, meta_y) ||
        !celdas_visitadas[meta_y][meta_x]) {
        return false;
    }

    uint8_t cola_x[MAPA_LADO * MAPA_LADO];
    uint8_t cola_y[MAPA_LADO * MAPA_LADO];
    int8_t padre_x[MAPA_LADO][MAPA_LADO];
    int8_t padre_y[MAPA_LADO][MAPA_LADO];
    uint8_t direccion_padre[MAPA_LADO][MAPA_LADO];
    bool vistos[MAPA_LADO][MAPA_LADO] = {};

    for (uint8_t iy = 0; iy < MAPA_LADO; iy++) {
        for (uint8_t ix = 0; ix < MAPA_LADO; ix++) {
            padre_x[iy][ix] = -1;
            padre_y[iy][ix] = -1;
            direccion_padre[iy][ix] = DIRECCION_DESCONOCIDA;
        }
    }

    uint8_t cabeza = 0;
    uint8_t cola = 0;
    cola_x[cola] = inicio_x;
    cola_y[cola++] = inicio_y;
    vistos[inicio_y][inicio_x] = true;

    while (cabeza < cola) {
        int8_t cx = cola_x[cabeza];
        int8_t cy = cola_y[cabeza++];
        if (cx == meta_x && cy == meta_y) break;

        for (uint8_t direccion = 0; direccion < 4; direccion++) {
            if ((salidas_abiertas[cy][cx] & mascaraDireccion(direccion)) == 0)
                continue;

            int8_t nx = cx + cambioX(direccion);
            int8_t ny = cy + cambioY(direccion);
            if (!dentroDelMapa(nx, ny) || !celdas_visitadas[ny][nx] ||
                vistos[ny][nx]) {
                continue;
            }

            vistos[ny][nx] = true;
            padre_x[ny][nx] = cx;
            padre_y[ny][nx] = cy;
            direccion_padre[ny][nx] = direccion;
            cola_x[cola] = nx;
            cola_y[cola++] = ny;
        }
    }

    if (!vistos[meta_y][meta_x]) return false;

    uint8_t reversa[MAX_CELDAS];
    uint8_t longitud = 0;
    int8_t cx = meta_x;
    int8_t cy = meta_y;

    while (cx != inicio_x || cy != inicio_y) {
        if (longitud >= MAX_CELDAS - 1) return false;

        uint8_t direccion = direccion_padre[cy][cx];
        if (direccion == DIRECCION_DESCONOCIDA) return false;

        reversa[longitud++] = direccion;
        int8_t siguiente_x = padre_x[cy][cx];
        int8_t siguiente_y = padre_y[cy][cx];
        cx = siguiente_x;
        cy = siguiente_y;
    }

    longitud_ruta_meta = longitud;
    for (uint8_t i = 0; i < longitud; i++) {
        ruta_meta[i] = reversa[longitud - i - 1];
    }
    return true;
}

void Pista1::iniciarSiguienteTramoRuta(RobotManager &robot) {
    if (fase == YENDO_A_META) {
        if (indice_ruta >= longitud_ruta_meta) {
            robot.drive.detener();
            robot.oled.mostrarEventoColor(F("ROJO"), false);
            Serial.println(F("Meta alcanzada. Iniciando regreso por la misma ruta."));
            inicio_pausa = millis();
            estado = PAUSA_META_FINAL;
            return;
        }

        iniciarMovimiento(robot, ruta_meta[indice_ruta], RUTA_A_META);
        return;
    }

    if (indice_ruta == 0) {
        robot.drive.detener();
        estado = META;
        Serial.println(F("Regreso al inicio completado."));
        return;
    }

    uint8_t direccion = (uint8_t)((ruta_meta[indice_ruta - 1] + 2U) & 3U);
    iniciarMovimiento(robot, direccion, RUTA_A_INICIO);
}

void Pista1::iniciarMovimiento(RobotManager &robot, uint8_t direccion,
                               TipoMovimiento tipo) {
    rumbo_destino = direccion & 3U;
    tipo_movimiento = tipo;
    destino_x = x + cambioX(rumbo_destino);
    destino_y = y + cambioY(rumbo_destino);

    if (!dentroDelMapa(destino_x, destino_y)) {
        robot.drive.detener();
        estado = SIN_RUTA;
        Serial.println(F("Destino fuera del mapa interno."));
        return;
    }

    int8_t diferencia = (int8_t)rumbo_destino - (int8_t)rumbo_celda;
    if (diferencia > 2) diferencia -= 4;
    if (diferencia < -2) diferencia += 4;

    if (diferencia != 0) {
        inicio_giro = millis();
        tiempo_limite_giro = abs(diferencia) == 2 ? 15000UL : 8000UL;
        robot.girarRelativo((float)diferencia * 90.0);
        estado = GIRANDO;
        return;
    }

    iniciarAvance(robot);
}

void Pista1::iniciarRetornoPorObstaculo(RobotManager &robot, unsigned long ahora) {
    duracion_retroceso_obstaculo = ahora - inicio_movimiento;
    if (duracion_retroceso_obstaculo < 100UL)
        duracion_retroceso_obstaculo = 100UL;

    rumbo_destino = (rumbo_celda + 2U) & 3U;
    regreso_obstaculo_pendiente = true;
    robot.girarRelativo(180.0);
    estado = GIRANDO;
    Serial.println(F("Obstaculo frontal: giro y regreso al centro de la celda."));
}

void Pista1::iniciarAvance(RobotManager &robot) {
    robot.drive.detener();
    rumbo_celda = rumbo_destino;

    inicio_movimiento = millis();
    tiempo_ultima_lectura = inicio_movimiento;
    estado = AVANZANDO;
}

void Pista1::completarMovimiento(RobotManager &robot) {
    robot.drive.detener();
    x = destino_x;
    y = destino_y;
    celdas_visitadas[y][x] = 1;

    if (tipo_movimiento == AVANCE_EXPLORACION) {
        if (profundidad >= PROFUNDIDAD_MAXIMA) {
            estado = SIN_RUTA;
            Serial.println(F("Limite de memoria de recorrido alcanzado."));
            return;
        }

        historial_x[profundidad] = (int8_t)(x - cambioX(rumbo_celda));
        historial_y[profundidad] = (int8_t)(y - cambioY(rumbo_celda));
        historial_direccion[profundidad] = rumbo_celda;
        profundidad++;
        celdas_unicas++;
        numero_casilla = celdas_unicas;
        robot.oled.setCasilla(numero_casilla);

        Serial.print(F("Celda nueva #"));
        Serial.print(numero_casilla);
        Serial.print(F(" ("));
        Serial.print(x);
        Serial.print(',');
        Serial.print(y);
        Serial.println(')');
    } else if (tipo_movimiento == RETORNO_EXPLORACION) {
        if (profundidad > 0) profundidad--;
        Serial.println(F("Regreso a la ultima bifurcacion."));
    } else if (tipo_movimiento == RUTA_A_META) {
        indice_ruta++;
        if (x == meta_x && y == meta_y) {
            robot.oled.mostrarEventoColor(F("ROJO"), false);
            inicio_pausa = millis();
            estado = PAUSA_META_FINAL;
            Serial.println(F("Meta alcanzada. Iniciando regreso por la misma ruta."));
            return;
        }
    } else if (tipo_movimiento == RUTA_A_INICIO) {
        if (indice_ruta > 0) indice_ruta--;
        if (indice_ruta == 0) {
            estado = META;
            Serial.println(F("Regreso al inicio completado."));
            return;
        }
    }

    inicio_espera_analisis = millis();
    estado = ESPERA_ANALISIS;

    if (fase != EXPLORANDO) {
        iniciarSiguienteTramoRuta(robot);
    }
}

void Pista1::procesarColor(RobotManager &robot, ColorDetectado color) {
    uint8_t indice_color = (uint8_t)color;
    uint16_t mascara_color = (uint16_t)(1U << indice_color);
    if (colores_detectados & mascara_color) return;

    if (color == ColorDetectado::ROJO) {
        if (!meta_encontrada && fase == EXPLORANDO) {
            meta_encontrada = true;
            if (estado == AVANZANDO) {
                meta_x = destino_x;
                meta_y = destino_y;
            } else {
                meta_x = x;
                meta_y = y;
            }
            Serial.print(F("Meta roja registrada en ("));
            Serial.print(meta_x);
            Serial.print(',');
            Serial.print(meta_y);
            Serial.println(')');
        }
    } else if (!esColorMarcador(color)) {
        return;
    }

    colores_detectados |= mascara_color;
    robot.drive.detener();
    estado_antes_pausa = estado;
    estado = PAUSA_COLOR;
    inicio_pausa = millis();
    robot.oled.mostrarEventoColor(ColorSensorComponent::nombre(color), false);

    Serial.print(F("Color detectado: "));
    Serial.println(ColorSensorComponent::nombre(color));
}

void Pista1::periodic(RobotManager &robot) {
    if (estado == META || estado == SIN_RUTA) {
        robot.drive.detener();
        return;
    }

    if (robot.colorCambio()) {
        procesarColor(robot, robot.getColor());
        if (estado == PAUSA_COLOR) return;
    }

    if (estado == PAUSA_COLOR) {
        robot.drive.detener();
        if (millis() - inicio_pausa >= TIEMPO_PAUSA_COLOR_MS) {
            unsigned long pausa = millis() - inicio_pausa;
            if (estado_antes_pausa == AVANZANDO) {
                inicio_movimiento += pausa;
                tiempo_ultima_lectura += pausa;
            } else if (estado_antes_pausa == GIRANDO) {
                inicio_giro += pausa;
            } else if (estado_antes_pausa == RETROCEDIENDO_OBSTACULO) {
                inicio_retroceso_obstaculo += pausa;
            }
            estado = estado_antes_pausa;
        }
        return;
    }

    if (estado == PAUSA_META_FINAL) {
        robot.drive.detener();
        if (millis() - inicio_pausa >= TIEMPO_PAUSA_META_MS) {
            fase = REGRESANDO_INICIO;
            indice_ruta = longitud_ruta_meta;
            iniciarSiguienteTramoRuta(robot);
        }
        return;
    }

    if (estado == RETROCEDIENDO_OBSTACULO) {
        unsigned long ahora = millis();
        const unsigned long transcurrido =
            ahora - inicio_retroceso_obstaculo;
        if (transcurrido >= duracion_retroceso_obstaculo) {
            robot.drive.detener();
            if (fase != EXPLORANDO) {
                estado = SIN_RUTA;
                Serial.println(F("Ruta final bloqueada; robot detenido."));
                return;
            }
            inicio_espera_analisis = ahora;
            estado = ESPERA_ANALISIS;
            return;
        }

        avanzarConCorreccion(robot, transcurrido,
                             duracion_retroceso_obstaculo, false);
        return;
    }

    if (estado == ESPERA_ANALISIS) {
        robot.drive.detener();
        if (millis() - inicio_espera_analisis >= TIEMPO_ESTABILIZAR_CASILLA_MS)
            estado = ANALIZANDO;
        return;
    }

    if (estado == ANALIZANDO) {
        if (fase == EXPLORANDO) analizarCasilla(robot);
        else iniciarSiguienteTramoRuta(robot);
        return;
    }

    if (estado == GIRANDO) {
        if (millis() - inicio_giro >= tiempo_limite_giro) {
            robot.drive.detener();
            estado = SIN_RUTA;
            Serial.println(F("Tiempo limite de giro; robot detenido."));
            return;
        }

        robot.controlarRumbo();
        if (robot.enRumbo()) {
            if (regreso_obstaculo_pendiente) {
                rumbo_celda = rumbo_destino;
                inicio_retroceso_obstaculo = millis();
                regreso_obstaculo_pendiente = false;
                estado = RETROCEDIENDO_OBSTACULO;
            } else {
                iniciarAvance(robot);
            }
        }
        return;
    }

    unsigned long ahora = millis();
    if (ahora - inicio_movimiento >= TIEMPO_MAXIMO_CASILLA_MS) {
        registrarSalida(x, y, rumbo_celda, false);
        robot.drive.detener();
        iniciarRetornoPorObstaculo(robot, ahora);
        return;
    }

    if (ahora - tiempo_ultima_lectura >= INTERVALO_ULTRASONICO_MS) {
        tiempo_ultima_lectura = ahora;
        float frente = medirFiltrado(sensor_frente);
        if (frente > 0.0 && frente <= DISTANCIA_SEGURIDAD_CM) {
            registrarSalida(x, y, rumbo_celda, false);
            robot.drive.detener();
            iniciarRetornoPorObstaculo(robot, ahora);
            return;
        }
    }

    const unsigned long transcurrido = ahora - inicio_movimiento;
    avanzarConCorreccion(robot, transcurrido, TIEMPO_CASILLA_MS, true);

    if (transcurrido >= TIEMPO_CASILLA_MS) {
        completarMovimiento(robot);
    }
}
