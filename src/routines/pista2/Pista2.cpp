#include "Pista2.hpp"

#include "RobotManager.hpp"
#include "constants/pinsConfig.h"

namespace {
constexpr uint8_t PIN_SERVO_PELOTA = 7;
constexpr uint8_t CANAL_COLOR_SECUNDARIO = 3;
constexpr int VELOCIDAD_AVANCE = 60;
constexpr int VELOCIDAD_LATERAL = 60;
constexpr int VELOCIDAD_GIRO = 38;
constexpr int VELOCIDAD_RECOLECCION = 60;
constexpr float DISTANCIA_INICIO_BOLA_CM = 42.0f;
constexpr float DISTANCIA_OBSTACULO_CM = 18.0f;
constexpr float UMBRAL_ABERTURA_CM = 48.0f;
constexpr float DISTANCIA_RETROCESO_CM = 20.0f;
constexpr unsigned long INTERVALO_SENSOR_MS = 80;
constexpr unsigned long TIMEOUT_GIRO_MS = 5000;
constexpr unsigned long TIMEOUT_RETROCESO_MS = 5000;
constexpr unsigned long TIMEOUT_FICHA_MS = 15000;
constexpr unsigned long PAUSA_FICHA_MS = 1500;
constexpr float DISTANCIA_SIN_ECO_CM = 250.0f;
constexpr float CORRECCION_RUMBO_MAX = 24.0f;

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
      distancia_referencia_frente(0.0f),
      objetivo_retroceso(0.0f),
      direccion_lateral(0),
      grados_salida(0),
      lecturas_retroceso(0),
      lecturas_fuera_blanco(0),
      ficha_actual_abandonada(false),
      marca_tiempo(0),
      tiempo_ultima_medicion(0),
      tiempo_movimiento_ficha(0),
      tiempo_actualizar_color_secundario(0) {}

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
    if (!compuerta.begin(PIN_SERVO_PELOTA, 20, 90)) {
        Serial.println(F("No se pudo iniciar el servo en pin 7."));
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
    estado = GIRO_GYRO;
}

void Pista2::completarGiro(RobotManager &robot) {
    if (robot.enRumbo()) {
        robot.drive.detener();
        estado = estado_despues_giro;
        marca_tiempo = millis();
        tiempo_ultima_medicion = 0;
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

void Pista2::seleccionarSalida() {
    const float frente = medirDistancia(sensor_frente);
    const float izquierda = medirDistancia(sensor_izquierdo);
    const float derecha = medirDistancia(sensor_derecho);
    const float maximo = max(frente, max(izquierda, derecha));

    if (maximo < UMBRAL_ABERTURA_CM) {
        grados_salida = 180;
    } else if (frente >= izquierda && frente >= derecha) {
        grados_salida = 0;
    } else if (izquierda >= derecha) {
        grados_salida = 90;
    } else {
        grados_salida = -90;
    }
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
            estado = P1_AVANZAR;
            marca_tiempo = millis();
        } else if (esVerde(color_1) || esVerde(color_2)) {
            estado = P2_AVANZAR;
            marca_tiempo = millis();
        } else if ((color_1 == color_2) && esFichaDireccion(color_1)) {
            iniciarFicha(color_1);
        } else if (millis() - marca_tiempo > 5000) {
            fallar(robot, F("No se pudo identificar la seccion por color"));
        } else {
            avanzarConRumbo(robot, VELOCIDAD_AVANCE);
        }
        return;

    case P1_AVANZAR:
        if (millis() - tiempo_ultima_medicion < INTERVALO_SENSOR_MS) {
            avanzarConRumbo(robot, VELOCIDAD_AVANCE);
            return;
        }
        tiempo_ultima_medicion = millis();
        if (medirDistancia(sensor_frente) <= DISTANCIA_INICIO_BOLA_CM) {
            robot.drive.detener();
            estado = P1_EVALUAR_DERECHA;
            return;
        }
        avanzarConRumbo(robot, VELOCIDAD_AVANCE);
        return;

    case P1_EVALUAR_DERECHA:
        if (medirDistancia(sensor_derecho) >= UMBRAL_ABERTURA_CM) {
            compuerta.abrir();
            iniciarGiro(robot, 90.0f, P1_MEDIR_RETROCESO);
        } else {
            estado = P1_ELEGIR_SALIDA;
        }
        return;

    case P1_MEDIR_RETROCESO:
        distancia_referencia_frente = medirDistancia(sensor_frente);
        if (distancia_referencia_frente >= DISTANCIA_SIN_ECO_CM - 1.0f) {
            fallar(robot, F("Sin eco frontal: no se puede medir retroceso"));
            return;
        }
        objetivo_retroceso =
            distancia_referencia_frente + DISTANCIA_RETROCESO_CM;
        lecturas_retroceso = 0;
        marca_tiempo = millis();
        estado = P1_RETROCEDER_PELOTA;
        return;

    case P1_RETROCEDER_PELOTA:
        if (millis() - marca_tiempo > TIMEOUT_RETROCESO_MS) {
            fallar(robot, F("Timeout al retroceder por la pelota"));
            return;
        }
        if (millis() - tiempo_ultima_medicion < INTERVALO_SENSOR_MS) {
            robot.drive.mover(
                -VELOCIDAD_RECOLECCION, 0,
                static_cast<int>(robot.correccionRumbo(CORRECCION_RUMBO_MAX)));
            return;
        }
        tiempo_ultima_medicion = millis();
        if (medirDistancia(sensor_frente) >= objetivo_retroceso) {
            if (++lecturas_retroceso >= 2) {
                robot.drive.detener();
                compuerta.contener();
                estado = P1_ELEGIR_SALIDA;
                return;
            }
        } else {
            lecturas_retroceso = 0;
        }
        robot.drive.mover(
            -VELOCIDAD_RECOLECCION, 0,
            static_cast<int>(robot.correccionRumbo(CORRECCION_RUMBO_MAX)));
        return;

    case P1_ELEGIR_SALIDA:
        seleccionarSalida();
        if (grados_salida == 0) {
            estado = P1_SALIR;
            marca_tiempo = millis();
        } else {
            iniciarGiro(robot, static_cast<float>(grados_salida), P1_SALIR);
        }
        return;

    case P1_SALIR:
        if (millis() - tiempo_ultima_medicion >= INTERVALO_SENSOR_MS) {
            tiempo_ultima_medicion = millis();
            leerParColores(robot, color_1, color_2);
            if (esVerde(color_1) || esVerde(color_2)) {
                robot.drive.detener();
                estado = P2_AVANZAR;
                marca_tiempo = millis();
                return;
            }
            if (medirDistancia(sensor_frente) <= DISTANCIA_OBSTACULO_CM) {
                robot.drive.detener();
                estado = P1_ELEGIR_SALIDA;
                return;
            }
        }
        avanzarConRumbo(robot, VELOCIDAD_AVANCE);
        return;

    case P2_AVANZAR:
        if (millis() - tiempo_ultima_medicion < INTERVALO_SENSOR_MS) {
            avanzarConRumbo(robot, VELOCIDAD_AVANCE);
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
        avanzarConRumbo(robot, VELOCIDAD_AVANCE);
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
        ficha_actual_abandonada = false;
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
            if (color_1 != ficha_actual && color_2 != ficha_actual) {
                ficha_actual_abandonada = true;
            }
            if (esVerde(color_1) && esVerde(color_2)) {
                robot.drive.detener();
                estado = TERMINADO;
                Serial.println(F("Pista B terminada"));
                return;
            }
            if (ficha_actual_abandonada && color_1 == color_2 &&
                esFichaDireccion(color_1)) {
                robot.drive.detener();
                iniciarFicha(color_1);
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
