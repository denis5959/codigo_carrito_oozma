#include <Arduino.h>
#include "RobotManager.hpp"

#if defined(PISTA_A)
  #include "routines/pista1/Pista1.hpp"
  Pista1 rutina;
#elif defined(PISTA_B)
  #include "routines/pista2/Pista2.hpp"
  Pista2 rutina;
#else
  #error "Selecciona PISTA_A o PISTA_B en platformio.ini"
#endif

RobotManager robot;

void setup() {
    robot.begin();
    rutina.begin(robot);
}

void loop() {
    if (!robot.actualizar()) return;
    rutina.periodic(robot);
    robot.terminarCiclo();
}