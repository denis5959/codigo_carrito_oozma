#include "Pista2.hpp"

#include "RobotManager.hpp"
#include "constants/pinsConfig.h"

namespace {
constexpr uint8_t CANAL_COLOR_SECUNDARIO = 3;
constexpr int VELOCIDAD_AVANCE = 60;
constexpr int VELOCIDAD_LATERAL = 60;
constexpr int VELOCIDAD_AVANCE_PISTA_B = 20;
constexpr float DISTANCIA_PARED_PISTA_A_CM = 8.0f;
constexpr float DISTANCIA_UNIDAD_CENTRO_CM = 30.0f;
constexpr unsigned long INTERVALO_SENSOR_MS = 80;
constexpr unsigned long TIMEOUT_GIRO_MS = 5000;
constexpr unsigned long TIMEOUT_IDENTIFICAR_SECCION_MS = 10000;
constexpr unsigned long TIMEOUT_TRAMO_PISTA_A_MS = 20000;
constexpr unsigned long TIMEOUT_CENTRO_PISTA_A_MS = 6000;
constexpr unsigned long TIMEOUT_CHECKPOINT_ROJO_MS = 20000;
constexpr unsigned long TIMEOUT_FICHA_MS = 15000;
constexpr unsigned long PAUSA_FICHA_MS = 1500;
constexpr unsigned long INTERVALO_REPORTE_COLOR_MS = 250;
constexpr unsigned long INTERVALO_PANTALLA_MS = 100;
constexpr float DISTANCIA_SIN_ECO_CM = 250.0f;
constexpr float CORRECCION_RUMBO_MAX = 24.0f;
constexpr uint8_t LECTURAS_GIRO_ESTABLE = 3;
constexpr float VELOCIDAD_GIRO_ESTABLE_DPS = 8.0f;

bool esBlanco(ColorDetectado color) {
    return color == ColorDetectado::BLANCO;
}

bool esVerde(ColorDetectado color) {
    return color == ColorDetectado::VERDE;
}
}

Pista2::Pista2()
    : estado(AVANCE_INICIAL),
      estado_despues_giro(AVANCE_INICIAL),
      sensor_frente(TRIG_ADELANTE, ECHO_ADELANTE),
      sensor_izquierdo(TRIG_IZQ, ECHO_IZQ),
      sensor_derecho(TRIG_DER, ECHO_DER),
      sensor_color_2(COLOR_ADDRESS, CANAL_COLOR_SECUNDARIO),
      ficha_actual(ColorDetectado::NINGUNO),
      ficha_pendiente(ColorDetectado::NINGUNO),
      objetivo_distancia_centro(0.0f),
      direccion_lateral(0),
      lecturas_pared(0),
      lecturas_centro(0),
      lecturas_checkpoint_rojo(0),
      lecturas_fuera_blanco(0),
      lecturas_giro_estable(0),
      marca_tiempo(0),
      tiempo_ultima_medicion(0),
      tiempo_movimiento_ficha(0),
      tiempo_actualizar_color_secundario(0),
      inicio_tramo_pista_a(0),
      tiempo_ultimo_reporte_color(0),
      tiempo_anterior_pantalla(0) {}

void Pista2::begin(RobotManager &robot) {
    sensor_frente.begin();
    sensor_izquierdo.begin();
    sensor_derecho.begin();
    if (!sensor_color_2.begin()) {
        Serial.println(F("No se encontro el TCS34725 secundario en canal 3."));
        robot.drive.detener();
        estado = ERROR;
        return;
    }
    robot.drive.detener();
    estado = AVANCE_INICIAL;
    marca_tiempo = millis();
    tiempo_ultima_medicion = 0;
    tiempo_actualizar_color_secundario = 0;
}

ColorDetectado Pista2::leerColorPrimario(const RobotManager &robot) const {
    return robot.getColor();
}

ColorDetectado Pista2::leerColorSecundario() const {
    return sensor_color_2.getColor();
}

const __FlashStringHelper *Pista2::nombrePartePista() const {
    const Estado estado_actual =
        estado == GIRO_GYRO ? estado_despues_giro : estado;

    switch (estado_actual) {
    case P1_TRAMO_1:
    case P1_TRAMO_2:
    case P1_TRAMO_CENTRO:
        return F("Seccion A");
    case P1_BUSCAR_CHECKPOINT_ROJO:
        return F("Checkpoint rojo");
    case P2_AVANZAR:
    case P2_MEDIR_LATERAL:
    case P2_DESLIZAR:
        return F("Seccion B");
    case P3_ESPERAR_FICHA:
    case P3_PAUSA_FICHA:
    case P3_SEGUIR_FICHA:
        return F("Seccion final");
    case TERMINADO:
        return F("Terminado");
    case ERROR:
        return F("Error");
    case AVANCE_INICIAL:
    case IDENTIFICAR_SECCION:
    case GIRO_GYRO:
        return F("Identificando seccion");
    }

    return F("Desconocida");
}

float Pista2::medirDistancia(UltrasonicSensor &sensor) {
    const float distancia = sensor.medirCm();
    if (distancia <= 0.0f) {
        return DISTANCIA_SIN_ECO_CM;
    }
    return distancia;
}

void Pista2::avanzarConRumbo(RobotManager &robot, int velocidad) {
    robot.drive.mover(
        velocidad, 0,
        static_cast<int>(robot.correccionRumbo(CORRECCION_RUMBO_MAX)));
}

void Pista2::iniciarGiro(RobotManager &robot, float grados,
                         Estado siguiente) {
    robot.drive.detener();
    robot.girarRelativo(grados);
    estado_despues_giro = siguiente;
    marca_tiempo = millis();
    lecturas_giro_estable = 0;
    estado = GIRO_GYRO;
}

void Pista2::completarGiro(RobotManager &robot) {
    if (robot.enRumbo() && fabs(robot.getVelocidadZ()) <=
                               VELOCIDAD_GIRO_ESTABLE_DPS) {
        if (lecturas_giro_estable < LECTURAS_GIRO_ESTABLE) {
            lecturas_giro_estable++;
        }
    } else {
        lecturas_giro_estable = 0;
    }

    if (lecturas_giro_estable >= LECTURAS_GIRO_ESTABLE) {
        robot.drive.detener();
        estado = estado_despues_giro;
        marca_tiempo = millis();
        tiempo_ultima_medicion = 0;
        lecturas_pared = 0;
        if (estado == P1_TRAMO_2) {
            inicio_tramo_pista_a = millis();
            Serial.println(F("Pista A: segundo tramo, avanzo hasta pared."));
        } else if (estado == P1_BUSCAR_CHECKPOINT_ROJO) {
            inicio_tramo_pista_a = millis();
            lecturas_checkpoint_rojo = 0;
            Serial.println(F("Pista A: giro a la izquierda completado; busco checkpoint rojo."));
        } else if (estado == P1_TRAMO_CENTRO) {
            const float distancia_inicio = sensor_frente.medirCm();
            if (distancia_inicio <= 0.0f) {
                fallar(robot, F("Pista A: sin eco al iniciar tramo al centro"));
                return;
            }
            objetivo_distancia_centro =
                max(DISTANCIA_PARED_PISTA_A_CM,
                    distancia_inicio - DISTANCIA_UNIDAD_CENTRO_CM);
            lecturas_centro = 0;
            inicio_tramo_pista_a = millis();
            Serial.print(F("Pista A: tramo a casilla central desde "));
            Serial.print(distancia_inicio, 1);
            Serial.print(F(" cm; objetivo "));
            Serial.print(objetivo_distancia_centro, 1);
            Serial.println(F(" cm."));
        }
        return;
    }

    if (millis() - marca_tiempo > TIMEOUT_GIRO_MS) {
        fallar(robot, F("Timeout de giro con IMU"));
        return;
    }
    robot.controlarRumbo();
}

bool Pista2::leerParColores(const RobotManager &robot,
                            ColorDetectado &primario,
                            ColorDetectado &secundario) {
    primario = leerColorPrimario(robot);
    secundario = leerColorSecundario();
    return primario != ColorDetectado::NINGUNO &&
           secundario != ColorDetectado::NINGUNO;
}

bool Pista2::esFichaDireccion(ColorDetectado color) const {
    return color == ColorDetectado::CIAN ||
           color == ColorDetectado::AMARILLO ||
           color == ColorDetectado::NARANJA ||
           color == ColorDetectado::MAGENTA;
}

void Pista2::iniciarFicha(ColorDetectado color) {
    ficha_pendiente = color;
    estado = P3_PAUSA_FICHA;
    marca_tiempo = millis();
}

void Pista2::fallar(RobotManager &robot,
                    const __FlashStringHelper *mensaje) {
    robot.drive.detener();
    Serial.println(mensaje);
    estado = ERROR;
}

void Pista2::periodic(RobotManager &robot) {
    ColorDetectado color_1 = ColorDetectado::NINGUNO;
    ColorDetectado color_2 = ColorDetectado::NINGUNO;

    if (millis() - tiempo_actualizar_color_secundario >= 60) {
        tiempo_actualizar_color_secundario = millis();
        sensor_color_2.actualizar();
    }

    if (millis() - tiempo_ultimo_reporte_color >=
        INTERVALO_REPORTE_COLOR_MS) {
        tiempo_ultimo_reporte_color = millis();
        Serial.print(F("Parte: "));
        Serial.print(nombrePartePista());
        Serial.print(F(" | color canal 2: "));
        Serial.print(ColorSensorComponent::nombre(leerColorPrimario(robot)));
        Serial.print(F(" | color canal 3: "));
        Serial.println(ColorSensorComponent::nombre(leerColorSecundario()));
    }

    if (millis() - tiempo_anterior_pantalla >=
        INTERVALO_PANTALLA_MS) {
        tiempo_anterior_pantalla = millis();
        robot.oled.mostrarEstadoPistaB(
            nombrePartePista(),
            ColorSensorComponent::nombre(leerColorPrimario(robot)),
            ColorSensorComponent::nombre(leerColorSecundario()));
    }

    switch (estado) {
    case AVANCE_INICIAL:
        if (millis() - marca_tiempo < 450) {
            avanzarConRumbo(robot, VELOCIDAD_AVANCE);
            return;
        }
        robot.drive.detener();
        estado = IDENTIFICAR_SECCION;
        marca_tiempo = millis();
        return;

    case IDENTIFICAR_SECCION:
        if (millis() - tiempo_ultima_medicion < INTERVALO_SENSOR_MS) {
            return;
        }
        tiempo_ultima_medicion = millis();
        leerParColores(robot, color_1, color_2);
        if (esBlanco(color_1) || esBlanco(color_2)) {
            estado = P1_TRAMO_1;
            marca_tiempo = millis();
            inicio_tramo_pista_a = millis();
            lecturas_pared = 0;
            Serial.println(F("Pista A: primer tramo, avanzo hasta pared."));
        } else if (esVerde(color_1) || esVerde(color_2)) {
            estado = P2_AVANZAR;
            marca_tiempo = millis();
        } else if ((color_1 == color_2) && esFichaDireccion(color_1)) {
            iniciarFicha(color_1);
        } else if (millis() - marca_tiempo >
                   TIMEOUT_IDENTIFICAR_SECCION_MS) {
            fallar(robot, F("No se pudo identificar la seccion por color"));
        } else {
            avanzarConRumbo(robot, VELOCIDAD_AVANCE);
        }
        return;

    case P1_TRAMO_1:
    case P1_TRAMO_2:
        if (millis() - tiempo_ultima_medicion < INTERVALO_SENSOR_MS) {
            avanzarConRumbo(robot, VELOCIDAD_AVANCE);
            return;
        }
        tiempo_ultima_medicion = millis();
        if (millis() - inicio_tramo_pista_a > TIMEOUT_TRAMO_PISTA_A_MS) {
            fallar(robot, F("Pista A: timeout esperando pared frontal"));
            return;
        }
        {
            const float distancia = sensor_frente.medirCm();
            if (distancia > 0.0f &&
                distancia <= DISTANCIA_PARED_PISTA_A_CM) {
                if (lecturas_pared < 2) lecturas_pared++;
            } else {
                lecturas_pared = 0;
            }
        }
        if (lecturas_pared >= 2) {
            robot.drive.detener();
            lecturas_pared = 0;
            if (estado == P1_TRAMO_1) {
                Serial.println(F("Pista A: pared detectada; giro 90 grados a la derecha."));
                iniciarGiro(robot, -90.0f, P1_TRAMO_2);
            } else {
                Serial.println(F("Pista A: segunda pared detectada; giro 90 grados a la derecha."));
                iniciarGiro(robot, -90.0f, P1_TRAMO_CENTRO);
            }
            return;
        }
        avanzarConRumbo(robot, VELOCIDAD_AVANCE);
        return;

    case P1_TRAMO_CENTRO:
        if (millis() - tiempo_ultima_medicion < INTERVALO_SENSOR_MS) {
            avanzarConRumbo(robot, VELOCIDAD_AVANCE);
            return;
        }
        tiempo_ultima_medicion = millis();
        if (millis() - inicio_tramo_pista_a > TIMEOUT_CENTRO_PISTA_A_MS) {
            fallar(robot, F("Pista A: timeout avanzando a casilla central"));
            return;
        }
        {
            const float distancia = sensor_frente.medirCm();
            if (distancia <= 0.0f) {
                fallar(robot, F("Pista A: se perdio eco en tramo a casilla central"));
                return;
            }
            if (distancia <= objetivo_distancia_centro) {
                if (lecturas_centro < 2) lecturas_centro++;
            } else {
                lecturas_centro = 0;
            }
        }
        if (lecturas_centro >= 2) {
            robot.drive.detener();
            Serial.println(F("Pista A: casilla central alcanzada; giro 90 grados a la izquierda."));
            iniciarGiro(robot, 90.0f, P1_BUSCAR_CHECKPOINT_ROJO);
            return;
        }
        avanzarConRumbo(robot, VELOCIDAD_AVANCE);
        return;

    case P1_BUSCAR_CHECKPOINT_ROJO:
        if (millis() - inicio_tramo_pista_a > TIMEOUT_CHECKPOINT_ROJO_MS) {
            fallar(robot, F("Pista A: timeout buscando checkpoint rojo"));
            return;
        }
        if (millis() - tiempo_ultima_medicion >= INTERVALO_SENSOR_MS) {
            tiempo_ultima_medicion = millis();
            leerParColores(robot, color_1, color_2);
            if (color_1 == ColorDetectado::ROJO ||
                color_2 == ColorDetectado::ROJO) {
                if (lecturas_checkpoint_rojo < 2) {
                    lecturas_checkpoint_rojo++;
                }
            } else {
                lecturas_checkpoint_rojo = 0;
            }

            if (lecturas_checkpoint_rojo >= 2) {
                robot.drive.detener();
                estado = P2_AVANZAR;
                marca_tiempo = millis();
                tiempo_ultima_medicion = 0;
                Serial.println(F("Checkpoint rojo detectado; inicio seccion B."));
                return;
            }

            const float distancia = sensor_frente.medirCm();
            if (distancia > 0.0f &&
                distancia <= DISTANCIA_PARED_PISTA_A_CM) {
                fallar(robot, F("Pista A: pared antes del checkpoint rojo"));
                return;
            }
        }
        avanzarConRumbo(robot, VELOCIDAD_AVANCE);
        return;

    case P2_AVANZAR:
        if (millis() - tiempo_ultima_medicion < INTERVALO_SENSOR_MS) {
            avanzarConRumbo(robot, VELOCIDAD_AVANCE_PISTA_B);
            return;
        }
        tiempo_ultima_medicion = millis();
        leerParColores(robot, color_1, color_2);
        if (esBlanco(color_1) && esBlanco(color_2)) {
            robot.drive.detener();
            estado = P2_MEDIR_LATERAL;
            return;
        }
        if (esFichaDireccion(color_1) && color_1 == color_2) {
            iniciarFicha(color_1);
            return;
        }
        avanzarConRumbo(robot, VELOCIDAD_AVANCE_PISTA_B);
        return;

    case P2_MEDIR_LATERAL: {
        const float izquierda = medirDistancia(sensor_izquierdo);
        const float derecha = medirDistancia(sensor_derecho);
        direccion_lateral = derecha >= izquierda ? 1 : -1;
        lecturas_fuera_blanco = 0;
        estado = P2_DESLIZAR;
        marca_tiempo = millis();
        return;
    }

    case P2_DESLIZAR:
        if (millis() - marca_tiempo > 8000) {
            fallar(robot, F("Timeout deslizando fuera de la zona blanca"));
            return;
        }
        if (millis() - tiempo_ultima_medicion >= INTERVALO_SENSOR_MS) {
            tiempo_ultima_medicion = millis();
            leerParColores(robot, color_1, color_2);
            if (!esBlanco(color_1) && !esBlanco(color_2)) {
                if (++lecturas_fuera_blanco >= 2) {
                    robot.drive.detener();
                    estado = P3_ESPERAR_FICHA;
                    marca_tiempo = millis();
                    return;
                }
            } else {
                lecturas_fuera_blanco = 0;
            }
        }
        robot.drive.mover(
            0, direccion_lateral * VELOCIDAD_LATERAL,
            static_cast<int>(robot.correccionRumbo(CORRECCION_RUMBO_MAX)));
        return;

    case P3_ESPERAR_FICHA:
        if (millis() - tiempo_ultima_medicion < INTERVALO_SENSOR_MS) {
            avanzarConRumbo(robot, VELOCIDAD_AVANCE);
            return;
        }
        tiempo_ultima_medicion = millis();
        leerParColores(robot, color_1, color_2);
        if (esVerde(color_1) && esVerde(color_2)) {
            robot.drive.detener();
            estado = TERMINADO;
            Serial.println(F("Pista B terminada"));
            return;
        }
        if (color_1 == color_2 && esFichaDireccion(color_1)) {
            robot.drive.detener();
            iniciarFicha(color_1);
            return;
        }
        avanzarConRumbo(robot, VELOCIDAD_AVANCE);
        return;

    case P3_PAUSA_FICHA:
        robot.drive.detener();
        if (millis() - marca_tiempo < PAUSA_FICHA_MS) {
            return;
        }
        ficha_actual = ficha_pendiente;
        tiempo_movimiento_ficha = millis();
        if (ficha_actual == ColorDetectado::CIAN) {
            robot.drive.mover(
                0, VELOCIDAD_LATERAL,
                static_cast<int>(robot.correccionRumbo(CORRECCION_RUMBO_MAX)));
        } else if (ficha_actual == ColorDetectado::AMARILLO) {
            robot.drive.mover(
                0, -VELOCIDAD_LATERAL,
                static_cast<int>(robot.correccionRumbo(CORRECCION_RUMBO_MAX)));
        } else if (ficha_actual == ColorDetectado::NARANJA) {
            avanzarConRumbo(robot, VELOCIDAD_AVANCE);
        } else if (ficha_actual == ColorDetectado::MAGENTA) {
            robot.drive.mover(
                -VELOCIDAD_AVANCE, 0,
                static_cast<int>(robot.correccionRumbo(CORRECCION_RUMBO_MAX)));
        }
        estado = P3_SEGUIR_FICHA;
        tiempo_ultima_medicion = 0;
        return;

    case P3_SEGUIR_FICHA:
        if (millis() - tiempo_movimiento_ficha > TIMEOUT_FICHA_MS) {
            fallar(robot, F("Timeout esperando la siguiente ficha"));
            return;
        }
        if (millis() - tiempo_ultima_medicion >= INTERVALO_SENSOR_MS) {
            tiempo_ultima_medicion = millis();
            leerParColores(robot, color_1, color_2);
            if (esVerde(color_1) && esVerde(color_2)) {
                robot.drive.detener();
                estado = TERMINADO;
                Serial.println(F("Pista B terminada"));
                return;
            }
            const bool nueva_ficha_sensor_1 =
                color_1 != ficha_actual && esFichaDireccion(color_1);
            const bool nueva_ficha_sensor_2 =
                color_2 != ficha_actual && esFichaDireccion(color_2);
            if (nueva_ficha_sensor_1 || nueva_ficha_sensor_2) {
                robot.drive.detener();
                iniciarFicha(nueva_ficha_sensor_1 ? color_1 : color_2);
                return;
            }
        }
        if (ficha_actual == ColorDetectado::CIAN) {
            robot.drive.mover(
                0, VELOCIDAD_LATERAL,
                static_cast<int>(robot.correccionRumbo(CORRECCION_RUMBO_MAX)));
        } else if (ficha_actual == ColorDetectado::AMARILLO) {
            robot.drive.mover(
                0, -VELOCIDAD_LATERAL,
                static_cast<int>(robot.correccionRumbo(CORRECCION_RUMBO_MAX)));
        } else if (ficha_actual == ColorDetectado::NARANJA) {
            avanzarConRumbo(robot, VELOCIDAD_AVANCE);
        } else {
            robot.drive.mover(
                -VELOCIDAD_AVANCE, 0,
                static_cast<int>(robot.correccionRumbo(CORRECCION_RUMBO_MAX)));
        }
        return;

    case GIRO_GYRO:
        completarGiro(robot);
        return;

    case TERMINADO:
    case ERROR:
        robot.drive.detener();
        return;
    }
}
