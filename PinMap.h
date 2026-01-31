/*==============================================================================
  File: PinMap.h
  Project: MegaHc160Storm32V3
  Description:
    Centralized pin mapping (AUDITED FROM REAL CODE)
    Source of truth: audit of .ino + ALL .cpp files
==============================================================================*/

#ifndef PINMAP_H
#define PINMAP_H

#include <Arduino.h>

/*==============================================================================
  CURRENT SENSOR (from CurrentSensor.cpp)
==============================================================================*/
#define PIN_CURRENT_LEFT        A0
#define PIN_CURRENT_RIGHT       A1

/*==============================================================================
  VOLTAGE SENSOR (from VoltageSensor.cpp)
==============================================================================*/
#define PIN_VOLTAGE_MAIN        A2

/*==============================================================================
  TEMPERATURE SENSOR (from TempSensor.cpp)
==============================================================================*/
#define PIN_TEMP_DRIVER_LEFT   A3
#define PIN_TEMP_DRIVER_RIGHT  A4

/*==============================================================================
  MOTOR OUTPUT (from MotorOutput.cpp)
==============================================================================*/
// LEFT MOTOR
#define PIN_MOTOR_L_PWM         5
#define PIN_MOTOR_L_DIR         22
#define PIN_MOTOR_L_EN          24

// RIGHT MOTOR
#define PIN_MOTOR_R_PWM         6
#define PIN_MOTOR_R_DIR         23
#define PIN_MOTOR_R_EN          25

/*==============================================================================
  SD CARD (from SDLogger.cpp)
==============================================================================*/
#define PIN_SD_CS               53
// SPI hardware pins on Mega 2560 (implicit)
// MOSI = 51, MISO = 50, SCK = 52

/*==============================================================================
  STORM32 CONTROLLER (from Storm32Controller.cpp)
==============================================================================*/
#define STORM32_SERIAL          Serial2
// Mega 2560 fixed pins: TX2 = D16, RX2 = D17

/*==============================================================================
  DEBUG SERIAL (from .ino)
==============================================================================*/
#define DEBUG_SERIAL            Serial

#endif // PINMAP_H
