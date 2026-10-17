#ifndef IR_SENSOR_HPP
#define IR_SENSOR_HPP

#include <Arduino.h>

class IRSensorComponent {
private:
    int pin;
    bool detecta_en_alto;

public:
    IRSensorComponent(int pin_sensor, bool activo_en_alto);
    void begin();
    bool detecta() const;
};

#endif