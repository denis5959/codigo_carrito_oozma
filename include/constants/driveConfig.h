#ifndef DRIVE_CONFIG_H
#define DRIVE_CONFIG_H

//offsets raw del gyro
const float OFFSET_GX = 237.754;
const float OFFSET_GY = 119.490;
const float OFFSET_GZ = 211.570;

const float OFFSET_AX = 364.628;
const float OFFSET_AY = -90.550;
const float OFFSET_AZ = -19.849;
const float SENSIBILIDAD_GIRO = 131.0; //rango +-250 dps
const float ZONA_MUERTA_DPS = 0.20;
const bool CALIBRAR_GIRO_AL_INICIO = false;
const int MUESTRAS_CALIBRACION_GIRO = 500;

//ajuste de velocidad lado derecho
const float MULTIPLICADOR_LADO_DERECHO = 0.87;

//debug pid por serial
const bool PID_DEBUG_SERIAL = true;


//invertir strafe si hace falta
const int STRAFE_SIGNO = 1;

#if defined(PISTA_A) //tanque
  #define NOMBRE_PISTA "PISTA A TANQUE"
  const bool ES_MECANUM = false;

  const float KP = 6.9;
  const float KI = 0.34;
  const float KD = 0.025;

  const int VELOCIDAD_MAXIMA = 75;
  const int VELOCIDAD_MINIMA = 24;

  const float TOLERANCIA_GRADOS = 2.0;
  const float ZONA_FRENADO_GRADOS = 12.0;

#elif defined(PISTA_B) //mecanum
  #define NOMBRE_PISTA "PISTA B MECANUM"
  const bool ES_MECANUM = true;

  const float KP = 2.0;
  const float KI = 0.;
  const float KD = 0;

  const int VELOCIDAD_MAXIMA = 48;
  const int VELOCIDAD_MINIMA = 30;

  const float TOLERANCIA_GRADOS = 6.0;
  const float ZONA_FRENADO_GRADOS = 24.0;
  const int SIGNO_GIRO_MECANUM = -1;

  const int VELOCIDAD_STRAFE = 100; //pwm derecha
  const int CORRECCION_MAXIMA_GIRO = 40; //limite giro

#else
  #error "Elige un environment: pista_a_tanque o pista_b_mecanum"
#endif

#endif