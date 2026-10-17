#ifndef PINS_CONFIG_H
#define PINS_CONFIG_H

#include <Arduino.h>

const int IN1_DEL_DER = 53;
const int IN2_DEL_DER = 52;
const int ENA_DEL_DER = 4; //pwm

const int IN3_DEL_IZQ = 51;
const int IN4_DEL_IZQ = 50;
const int ENB_DEL_IZQ = 5; //pwm

const int IN1_TRA_DER = 22;
const int IN2_TRA_DER = 23;
const int ENA_TRA_DER = 2; //pwm

const int IN3_TRA_IZQ = 24;
const int IN4_TRA_IZQ = 25;
const int ENB_TRA_IZQ = 3; //pwm

#if defined(PISTA_A)
const int IR_PISO_1 = 42;
#else
const int IR_PISO_1 = 44;
#endif
const int IR_PISO_2 = 43;

#if defined(PISTA_A)
const bool IR_DETECTA_EN_ALTO = false;
#else
const bool IR_DETECTA_EN_ALTO = true;
#endif

const int TRIG_ADELANTE = 45;
const int ECHO_ADELANTE = 44;
const int TRIG_IZQ = 48;
const int ECHO_IZQ = 49;
const int TRIG_DER = 46;
const int ECHO_DER = 47;

#if defined(CHASIS_TANQUE)
  const bool INV_DEL_DER = false;
  const bool INV_DEL_IZQ = false;
  const bool INV_TRA_DER = false;
  const bool INV_TRA_IZQ = false;
#elif defined(CHASIS_MECANUM)
  const bool INV_DEL_DER = false;
  const bool INV_DEL_IZQ = false;
  const bool INV_TRA_DER = false;
  const bool INV_TRA_IZQ = false;
#else
  #error "Elige un environment: pista_a_tanque o pista_b_mecanum"
#endif

const byte MPU_ADDRESS = 0x68; //ad0 a gnd
const byte OLED_ADDRESS = 0x3C;
const int OLED_RESET = -1;

const byte COLOR_ADDRESS = 0x29;

#endif