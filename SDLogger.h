// ========================================================================================
// SDLogger.h  
// ========================================================================================
#pragma once

#include <Arduino.h>
#include <SD.h>
#include "PinMap.h"
#include "SystemTypes.h"   // SystemState, DriveState, BladeState, SafetyState, DriveEvent

// ========================================================================================
// CONFIG (LOGGER BEHAVIOR)
// ========================================================================================
#define SD_LOG_PERIOD_MS        50      // 20 Hz
#define SD_LOG_BUFFER_SIZE      6
#define SD_LOG_WRITE_BUDGET_US  2000    // จำกัดเวลาการเขียน SD ต่อครั้ง (us)

// ========================================================================================
// LOG RECORD STRUCT (PLAIN, POD, FIELD-SAFE)
// ========================================================================================
struct SDLogRecord {
  uint32_t time_ms;

  float    curA[4];
  float    curMax;

  int16_t  tempDriverL;
  int16_t  tempDriverR;

  float    engineVolt;

  int16_t  pwmL;
  int16_t  pwmR;

  uint8_t  systemState;
  uint8_t  driveState;
  uint8_t  bladeState;
  uint8_t  safetyState;
  uint8_t  driveEvent;

  uint8_t  faultLatched;
};

// ========================================================================================
// PUBLIC API
// ========================================================================================
namespace SDLogger {

  // --------------------------------------------------
  // INIT / RECOVER
  // --------------------------------------------------
  bool begin();

  // --------------------------------------------------
  // FAST PATH (NO SD ACCESS)
  // --------------------------------------------------
  void log(uint32_t now);

  // --------------------------------------------------
  // WARN / EVENT LOG (EVENT-BASED, IMMEDIATE)
  // ใช้กับ SafetyState::WARN
  // --------------------------------------------------
  void logWarn(uint32_t now,
               const float curA[4],
               int16_t tempDriverL,
               int16_t tempDriverR,
               SafetyState safety);

  // --------------------------------------------------
  // SLOW PATH (SD ACCESS, TIME-BUDGETED)
  // --------------------------------------------------
  void flush();

  // --------------------------------------------------
  // STATUS
  // --------------------------------------------------
  bool isReady();
  bool bufferFull();
  uint8_t pending();

}  // namespace SDLogger
