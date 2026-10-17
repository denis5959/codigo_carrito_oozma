#include "RobotManager.hpp"

RobotManager::RobotManager()
    : drive(Motor(IN1_DEL_DER, IN2_DEL_DER, ENA_DEL_DER, INV_DEL_DER),
            Motor(IN3_DEL_IZQ, IN4_DEL_IZQ, ENB_DEL_IZQ, INV_DEL_IZQ),
            Motor(IN1_TRA_DER, IN2_TRA_DER, ENA_TRA_DER, INV_TRA_DER),
            Motor(IN3_TRA_IZQ, IN4_TRA_IZQ, ENB_TRA_IZQ, INV_TRA_IZQ)),
      mpu(MPU_ADDRESS),
      colorSensor(COLOR_ADDRESS),
      oled(128, 64, &Wire, OLED_RESET),
      irPiso1(IR_PISO_1, IR_DETECTA_EN_ALTO),
      irPiso2(IR_PISO_2, IR_DETECTA_EN_ALTO),
      pid(KP, KI, KD) {

    angulo_objetivo = 0.0;
    color_anterior = ColorDetectado::NINGUNO;
    color_cambio = false;
    tiempo_anterior_oled = 0;
    tiempo_anterior_pid = millis();
    tiempo_anterior_color = 0;
}

void RobotManager::begin() {
    Serial.begin(115200);
    Wire.begin();

    drive.begin();
    irPiso1.begin();
    irPiso2.begin();

    if (!oled.begin(OLED_ADDRESS)) {
        Serial.println(F("No se encontro OLED; se continua sin pantalla."));
    } else {
        oled.setTitulo(F(NOMBRE_PISTA));
        oled.mostrarInicio();
    }

    if (!colorSensor.begin()) {
        Serial.println(F("No se encontro TCS34725 en canal 2; el robot inicia sin deteccion de color."));
        oled.mostrarError(F("TCS34725"));
    }

    mpu.begin();

    angulo_objetivo = 0.0;
    pid.reset();
    tiempo_anterior_pid = millis();
    tiempo_anterior_color = millis();

    Serial.print(F("Robot listo: "));
    Serial.println(F(NOMBRE_PISTA));
    delay(800);
}

float RobotManager::normalizar360(float angulo) const {
    while (angulo >= 360.0) angulo -= 360.0;
    while (angulo < 0.0) angulo += 360.0;
    return angulo;
}

float RobotManager::calcularErrorAngular(float objetivo, float actual) const {
    float error = normalizar360(objetivo) - normalizar360(actual);
    if (error > 180.0) error -= 360.0;
    if (error < -180.0) error += 360.0;
    return error;
}

bool RobotManager::actualizar() {
    for (uint8_t intento = 0; intento < 3; intento++) {
        if (mpu.actualizar()) return true;
        delayMicroseconds(500);
    }

    Serial.println(F("Error leyendo MPU en canal I2C 5, direccion 0x68 tras 3 intentos; motores detenidos."));
    oled.mostrarError(F("MPU: revisar I2C"));
    drive.detener();
    delay(100);
    return false;
}

void RobotManager::terminarCiclo() {
    if (PID_DEBUG_SERIAL) {
        Serial.print(F("Rumbo: "));
        Serial.print(getRumbo(), 2);
        Serial.print(F(" | Objetivo: "));
        Serial.print(angulo_objetivo, 1);
        Serial.print(F(" | Error: "));
        Serial.print(getError(), 2);
        Serial.print(F(" | Vel Z: "));
        Serial.println(getVelocidadZ(), 2);
    }

    color_cambio = false;
    if (millis() - tiempo_anterior_color >= COLOR_INTERVALO_MS) {
        tiempo_anterior_color = millis();
        colorSensor.actualizar();

        ColorDetectado color = colorSensor.getColor();
        if (color != color_anterior) {
            color_anterior = color;
            color_cambio = true;
        }

        if (COLOR_DEBUG_SERIAL) {
            Serial.print(F("R:")); Serial.print(colorSensor.getRojo());
            Serial.print(F(" G:")); Serial.print(colorSensor.getVerde());
            Serial.print(F(" B:")); Serial.print(colorSensor.getAzul());
            Serial.print(F(" C:")); Serial.print(colorSensor.getClaro());
            Serial.print(F(" -> ")); Serial.println(ColorSensorComponent::nombre(color));
        }
    }

    if (millis() - tiempo_anterior_oled >= 100) {
        tiempo_anterior_oled = millis();
        oled.mostrarEstado(mpu.getAnguloZ(), angulo_objetivo, mpu.getVelocidadZ(),
                           ColorSensorComponent::nombre(colorSensor.getColor()));
    }

    delay(20);
}

void RobotManager::fijarRumbo(float grados) {
    angulo_objetivo = normalizar360(grados);
    pid.reset();
    tiempo_anterior_pid = millis();

    Serial.print(F("NUEVO OBJETIVO: "));
    Serial.print(angulo_objetivo, 1);
    Serial.println(F(" grados"));
}

void RobotManager::girarRelativo(float grados) {
#if defined(PISTA_A)
    fijarRumbo(mpu.getAnguloZ() + grados);
#else
    fijarRumbo(angulo_objetivo + grados);
#endif
}

void RobotManager::controlarRumbo() {
    float error = getError();

    unsigned long ahora = millis();
    float dt_pid = (ahora - tiempo_anterior_pid) / 1000.0;
    tiempo_anterior_pid = ahora;

    if (dt_pid > 0.2) {
        pid.reset();
        dt_pid = 0.02;
    }

    if (abs(error) <= TOLERANCIA_GRADOS) {
        drive.detener();
        pid.reposo(error);
        return;
    }

    float salida = pid.calcular(error, dt_pid);

    float proporcion = abs(error) / ZONA_FRENADO_GRADOS;
    proporcion = constrain(proporcion, 0.0, 1.0);
    float limite_pwm = VELOCIDAD_MINIMA + proporcion * (VELOCIDAD_MAXIMA - VELOCIDAD_MINIMA);

    salida = constrain(salida, -limite_pwm, limite_pwm);
    if (abs(salida) < VELOCIDAD_MINIMA) {
        salida = (salida >= 0) ? VELOCIDAD_MINIMA : -VELOCIDAD_MINIMA;
    }

#if defined(PISTA_A)
    drive.girarSobreEje(-(int)salida);
#else
    drive.girarSobreEje((int)salida);
#endif
}

float RobotManager::correccionRumbo(float limite) {
    float error = getError();

    unsigned long ahora = millis();
    float dt_pid = (ahora - tiempo_anterior_pid) / 1000.0;
    tiempo_anterior_pid = ahora;

    if (dt_pid > 0.2) {
        pid.reset();
        dt_pid = 0.02;
    }

    if (abs(error) <= TOLERANCIA_GRADOS) {
        pid.reposo(error);
        return 0.0;
    }

    float salida = pid.calcular(error, dt_pid);
    return constrain(salida, -limite, limite);
}

bool RobotManager::enRumbo() const {
    return abs(getError()) <= TOLERANCIA_GRADOS;
}

float RobotManager::getRumbo() const {
    return normalizar360(mpu.getAnguloZ());
}

float RobotManager::getObjetivo() const {
    return angulo_objetivo;
}

float RobotManager::getError() const {
    return calcularErrorAngular(angulo_objetivo, mpu.getAnguloZ());
}

float RobotManager::getVelocidadZ() const {
    return mpu.getVelocidadZ();
}

ColorDetectado RobotManager::getColor() const {
    return colorSensor.getColor();
}

bool RobotManager::colorCambio() const {
    return color_cambio;
}