#ifndef PISTA2_HPP
#define PISTA2_HPP

#include <Arduino.h>

class RobotManager;

class Pista2 {
public:
    Pista2();
    void begin(RobotManager &robot);
    void periodic(RobotManager &robot);
};

#endif