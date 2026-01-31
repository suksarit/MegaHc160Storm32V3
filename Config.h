/*==============================================================================
  File: Config.h
  Project: MegaHc160Storm32V3
  Description:
    System configuration extracted from behavior of real code.
    No speculative parameters included.
==============================================================================*/

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

/*==============================================================================
  SERIAL CONFIGURATION
==============================================================================*/
#define DEBUG_BAUDRATE          115200
#define STORM32_BAUDRATE        115200

/*==============================================================================
  ADC CONFIGURATION
==============================================================================*/
#define ADC_RESOLUTION          10      // Arduino Mega default
#define ADC_MAX_VALUE           1023

/*==============================================================================
  MOTOR CONTROL CONFIGURATION
==============================================================================*/
#define MOTOR_PWM_MIN           0
#define MOTOR_PWM_MAX           255

/*==============================================================================
  IBUS / REMOTE CONTROL
==============================================================================*/
// NOTE:
// Channel numbers inferred from DriveTarget usage pattern
// IBUS decoding itself happens outside this file
#define IBUS_MIN_VALUE          1000
#define IBUS_CENTER_VALUE       1500
#define IBUS_MAX_VALUE          2000
#define IBUS_DEADZONE           30

#define IBUS_CH_STEERING        1
#define IBUS_CH_THROTTLE        2
#define IBUS_CH_SPEED           3
#define IBUS_CH_ARM             4

/*==============================================================================
  SAFETY / LIMIT VALUES
==============================================================================*/
// These limits are consumed by SafetyManager logic
#define LIMIT_VOLTAGE_MIN       20.0f
#define LIMIT_VOLTAGE_MAX       30.0f
#define LIMIT_CURRENT_MAX       80.0f
#define LIMIT_TEMP_MAX          85.0f

#define SIGNAL_LOSS_TIMEOUT_MS  3000

/*==============================================================================
  SYSTEM TIMING
==============================================================================*/
#define MAIN_LOOP_TARGET_HZ     200

/*==============================================================================
  LOGGING
==============================================================================*/
#define SD_LOG_ENABLE           1
#define SD_LOG_FILENAME         "RUNLOG.CSV"

/*==============================================================================
  DEBUG MACROS
==============================================================================*/
#define DEBUG_ENABLE            1

#if DEBUG_ENABLE
  #define DBG(x)    DEBUG_SERIAL.print(x)
  #define DBGLN(x)  DEBUG_SERIAL.println(x)
#else
  #define DBG(x)
  #define DBGLN(x)
#endif

#endif // CONFIG_H
