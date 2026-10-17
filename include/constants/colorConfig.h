#ifndef COLOR_CONFIG_H
#define COLOR_CONFIG_H

#include <Arduino.h>
#include <Adafruit_TCS34725.h>

const uint8_t COLOR_TIEMPO_INTEGRACION = TCS34725_INTEGRATIONTIME_50MS;
const tcs34725Gain_t COLOR_GANANCIA = TCS34725_GAIN_4X;

const unsigned long COLOR_INTERVALO_MS = 80;

const uint16_t COLOR_CLARO_MAX_NEGRO = 300; //negro
const uint16_t COLOR_CLARO_MIN_BLANCO = 6000; //blanco

const float COLOR_FRACCION_DOMINANTE = 0.42;

const byte COLOR_LECTURAS_CONFIRMACION = 3;

//debug lecturas
const bool COLOR_DEBUG_SERIAL = false;

#endif