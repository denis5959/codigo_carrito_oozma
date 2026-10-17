#include "Pista2.hpp"
#include "RobotManager.hpp"

Pista2::Pista2() {}

void Pista2::begin(RobotManager &robot) {
    robot.fijarRumbo(robot.getRumbo());
}

void Pista2::periodic(RobotManager &robot) {
    bool hay_senal = robot.irPiso1.detecta() && robot.irPiso2.detecta();

    if (hay_senal) {
        int giro = (int)robot.correccionRumbo(CORRECCION_MAXIMA_GIRO);
        robot.drive.mover(0, VELOCIDAD_STRAFE, giro);
    } else {
        robot.controlarRumbo();
    }
}