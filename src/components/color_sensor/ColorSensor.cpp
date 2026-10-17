#include "ColorSensor.hpp"
#include "constants/colorConfig.h"
#include "constants/i2cConfig.h"

ColorSensorComponent::ColorSensorComponent(byte addr)
    : tcs(COLOR_TIEMPO_INTEGRACION, COLOR_GANANCIA), address(addr) {
    rojo = 0; verde = 0; azul = 0; claro = 0;
    candidato = ColorDetectado::NINGUNO;
    confirmado = ColorDetectado::NINGUNO;
    conteo_candidato = 0;
    disponible = false;
}

bool ColorSensorComponent::begin() {
    seleccionarCanalI2C(COLOR_SENSOR_I2C_CHANNEL);
    disponible = tcs.begin(address, &Wire);
    return disponible;
}

ColorDetectado ColorSensorComponent::clasificar(uint16_t r, uint16_t g, uint16_t b, uint16_t c) const {
    if (c < COLOR_CLARO_MAX_NEGRO) return ColorDetectado::NEGRO;

    if (c == 0) return ColorDetectado::NINGUNO;

    float rojo_norm = (float)r / c * 255.0;
    float verde_norm = (float)g / c * 255.0;
    float azul_norm = (float)b / c * 255.0;
    float mayor = max(rojo_norm, max(verde_norm, azul_norm));
    float menor = min(rojo_norm, min(verde_norm, azul_norm));

    if (mayor - menor < 45.0) return ColorDetectado::BLANCO;

    if (rojo_norm >= verde_norm && rojo_norm >= azul_norm) {
        if (rojo_norm <= 0.0) return ColorDetectado::NINGUNO;
        if (azul_norm > 0.4 * rojo_norm && azul_norm > 0.6 * verde_norm)
            return ColorDetectado::MAGENTA;
        if (verde_norm > 0.7 * rojo_norm) return ColorDetectado::AMARILLO;

        float naranja_ratio = (verde_norm - azul_norm) / rojo_norm;
        if (naranja_ratio > 0.16) return ColorDetectado::NARANJA;
        return ColorDetectado::ROJO;
    }

    if (verde_norm >= rojo_norm && verde_norm >= azul_norm) {
        if (rojo_norm > 0.75 * verde_norm) return ColorDetectado::AMARILLO;
        if (azul_norm > 0.75 * verde_norm) return ColorDetectado::CIAN;
        return ColorDetectado::VERDE;
    }

    if (verde_norm > 0.75 * azul_norm) return ColorDetectado::CIAN;
    return ColorDetectado::AZUL;
}

void ColorSensorComponent::actualizar() {
    if (!disponible) return;

    seleccionarCanalI2C(COLOR_SENSOR_I2C_CHANNEL);
    tcs.getRawData(&rojo, &verde, &azul, &claro);

    ColorDetectado crudo = clasificar(rojo, verde, azul, claro);

    if (crudo == candidato) {
        if (conteo_candidato < 255) conteo_candidato++;
    } else {
        candidato = crudo;
        conteo_candidato = 1;
    }

    if (conteo_candidato >= COLOR_LECTURAS_CONFIRMACION) {
        confirmado = candidato;
    }
}

ColorDetectado ColorSensorComponent::getColor() const { return confirmado; }
uint16_t ColorSensorComponent::getRojo() const { return rojo; }
uint16_t ColorSensorComponent::getVerde() const { return verde; }
uint16_t ColorSensorComponent::getAzul() const { return azul; }
uint16_t ColorSensorComponent::getClaro() const { return claro; }

const __FlashStringHelper *ColorSensorComponent::nombre(ColorDetectado color) {
    switch (color) {
        case ColorDetectado::NEGRO:  return F("NEGRO");
        case ColorDetectado::BLANCO: return F("BLANCO");
        case ColorDetectado::ROJO:   return F("ROJO");
        case ColorDetectado::VERDE:  return F("VERDE");
        case ColorDetectado::AZUL:   return F("AZUL");
        case ColorDetectado::AMARILLO: return F("AMARILLO");
        case ColorDetectado::CIAN:   return F("CIAN");
        case ColorDetectado::NARANJA: return F("NARANJA");
        case ColorDetectado::MAGENTA: return F("MAGENTA");
        default:                     return F("NADA");
    }
}