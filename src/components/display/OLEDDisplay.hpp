#ifndef OLED_DISPLAY_HPP
#define OLED_DISPLAY_HPP

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

class OLEDDisplayComponent {
private:
    Adafruit_SSD1306 display;
    const __FlashStringHelper *titulo;
    const __FlashStringHelper *ultimo_color;
    unsigned long fin_evento_color;
    bool evento_meta;
    uint16_t numero_casilla;
    bool disponible;

public:
    OLEDDisplayComponent(int screen_width, int screen_height, TwoWire *wire, int reset_pin);
    bool begin(uint8_t address);
    void setTitulo(const __FlashStringHelper *nuevo_titulo);
    void setCasilla(uint16_t casilla);
    void mostrarEstado(float rumbo_actual, float objetivo, float vel_z, const __FlashStringHelper *color);
    void mostrarEventoColor(const __FlashStringHelper *color, bool es_meta);
    void mostrarError(const __FlashStringHelper *mensaje);
    void mostrarInicio();
};

#endif