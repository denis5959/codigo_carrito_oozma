#ifndef COLOR_SENSOR_HPP
#define COLOR_SENSOR_HPP

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

enum class ColorDetectado : uint8_t {
    NINGUNO,
    NEGRO,
    BLANCO,
    ROJO,
    VERDE,
    AZUL,
    AMARILLO,
    CIAN,
    NARANJA,
    MAGENTA
};

class ColorSensorComponent {
private:
    Adafruit_TCS34725 tcs;
    byte address;
    byte channel;

    uint16_t rojo, verde, azul, claro;

    ColorDetectado candidato;
    ColorDetectado confirmado;
    byte conteo_candidato;
    bool disponible;

    ColorDetectado clasificar(uint16_t r, uint16_t g, uint16_t b, uint16_t c) const;

public:
    ColorSensorComponent(byte addr, byte i2c_channel = 2);
    bool begin();
    void actualizar();

    ColorDetectado getColor() const;
    uint16_t getRojo() const;
    uint16_t getVerde() const;
    uint16_t getAzul() const;
    uint16_t getClaro() const;

    static const __FlashStringHelper *nombre(ColorDetectado color);
};

#endif