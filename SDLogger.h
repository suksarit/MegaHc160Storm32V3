// ========================================================================================
// SDLogger.h  (PHASE 1)
// ========================================================================================
#pragma once

#include <Arduino.h>
#include <SD.h>
#include "PinMap.h"
#include "SystemTypes.h"

// ========================================================================================
// CONFIG
// ========================================================================================
#define SD_LOG_PERIOD_MS        50
#define SD_LOG_BUFFER_SIZE      6
#define SD_LOG_WRITE_BUDGET_US  2000

// ========================================================================================
// LOG RECORD STRUCT (PHASE 1)
// ========================================================================================
struct SDLogRecord {

  uint32_t time_ms;
  uint16_t loop_dt_ms;     // ★ NEW: loop health

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
  uint8_t  faultCode;      // ★ NEW
};

// ========================================================================================
// PUBLIC API
// ========================================================================================
namespace SDLogger {

  bool begin();
  void log(uint32_t now);
  void logWarn(uint32_t now,
               const float curA[4],
               int16_t tempDriverL,
               int16_t tempDriverR,
               SafetyState safety);
  void flush();

  bool isReady();
  bool bufferFull();
  uint8_t pending();
}
