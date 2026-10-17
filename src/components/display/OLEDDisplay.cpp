#include "OLEDDisplay.hpp"
#include "constants/i2cConfig.h"

OLEDDisplayComponent::OLEDDisplayComponent(int screen_width, int screen_height, TwoWire *wire, int reset_pin)
    : display(screen_width, screen_height, wire, reset_pin), titulo(nullptr),
      ultimo_color(F("NADA")), fin_evento_color(0), evento_meta(false),
      numero_casilla(1), disponible(false) {}

bool OLEDDisplayComponent::begin(uint8_t address) {
    seleccionarCanalI2C(OLED_I2C_CHANNEL);
    disponible = display.begin(SSD1306_SWITCHCAPVCC, address);
    return disponible;
}

void OLEDDisplayComponent::setTitulo(const __FlashStringHelper *nuevo_titulo) {
    titulo = nuevo_titulo;
}

void OLEDDisplayComponent::setCasilla(uint16_t casilla) {
    numero_casilla = casilla;
}

void OLEDDisplayComponent::mostrarInicio() {
    if (!disponible) return;

    seleccionarCanalI2C(OLED_I2C_CHANNEL);
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    if (titulo) {
        display.setCursor(0, 10);
        display.println(titulo);
    }
    display.setCursor(0, 30);
    display.println(F("Iniciando..."));
    display.display();
}

void OLEDDisplayComponent::mostrarEstado(float rumbo_actual, float objetivo, float vel_z, const __FlashStringHelper *color) {
    if (!disponible) return;

    seleccionarCanalI2C(OLED_I2C_CHANNEL);
    if (evento_meta || (fin_evento_color != 0 && millis() < fin_evento_color)) {
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println(titulo ? titulo : F("ROBOT"));
        display.drawLine(0, 11, 127, 11, SSD1306_WHITE);
        display.setCursor(0, 19);
        display.print(F("CASILLA "));
        display.println(numero_casilla);
        display.setCursor(0, 31);
        if (evento_meta) {
            display.println(F("META ROJA"));
            display.setTextSize(2);
            display.setCursor(0, 47);
            display.println(F("OBJETIVO"));
        } else {
            display.println(F("COLOR DETECTADO:"));
            display.setTextSize(2);
            display.setCursor(0, 44);
            display.println(ultimo_color);
        }
        display.display();
        return;
    }
    fin_evento_color = 0;

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    display.setTextSize(1);
    display.setCursor(0, 0);
    if (titulo) display.println(titulo);
    display.drawLine(0, 11, 127, 11, SSD1306_WHITE);

    display.setTextSize(2);
    display.setCursor(0, 18);
    display.print(F("Rumbo: "));
    while (rumbo_actual >= 360.0) rumbo_actual -= 360.0;
    while (rumbo_actual < 0.0) rumbo_actual += 360.0;
    display.print(rumbo_actual, 1);

    display.setTextSize(1);
    display.setCursor(0, 40);
    display.print(F("Casilla "));
    display.print(numero_casilla);
    display.print(F(" Rumbo "));
    display.print(objetivo, 0);
    display.println(F(" deg"));

    display.setCursor(0, 52);
    display.print(F("Ultimo color: "));
    display.print(ultimo_color ? ultimo_color : color);

    display.display();
}

void OLEDDisplayComponent::mostrarEventoColor(const __FlashStringHelper *color, bool es_meta) {
    ultimo_color = color;
    evento_meta = es_meta;
    fin_evento_color = es_meta ? 0 : millis() + 8000UL;
    if (!disponible) return;

    mostrarEstado(0.0, 0.0, 0.0, color);
}

void OLEDDisplayComponent::mostrarError(const __FlashStringHelper *mensaje) {
    if (!disponible) return;

    seleccionarCanalI2C(OLED_I2C_CHANNEL);
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 20);
    display.println(F("ERROR:"));
    display.setCursor(0, 35);
    display.println(mensaje);
    display.display();
}