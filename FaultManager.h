// ========================================================================================
// FaultManager.h
// ========================================================================================
#pragma once

#include <Arduino.h>
#include "SystemTypes.h"

// ============================================================================
// FAULT CODE (SINGLE SOURCE OF TRUTH)
// ============================================================================
enum class FaultCode : uint8_t {
  NONE,

  // ===== Communication =====
  IBUS_LOST,
  COMMS_TIMEOUT,
  LOGIC_WATCHDOG,

  // ===== Sensor =====
  CUR_SENSOR_FAULT,
  VOLT_SENSOR_FAULT,
  TEMP_SENSOR_FAULT,

  // ===== Power / Thermal =====
  OVER_CURRENT,
  OVER_TEMP,

  // ===== Timing =====
  DRIVE_TIMEOUT,
  BLADE_TIMEOUT,
  LOOP_OVERRUN
};

// ============================================================================
// GLOBAL FAULT STATE
// ============================================================================
extern FaultCode activeFault;
extern bool faultLatched;

// ============================================================================
// PUBLIC API
// ============================================================================

// latch fault ครั้งเดียว (ไม่ overwrite)
void latchFault(FaultCode code);

// ตัดของจริงทั้งหมด (motor + blade + ignition + starter)
// ❗ ต้องเรียกจาก loop exit เท่านั้น
void handleFaultImmediateCut();
