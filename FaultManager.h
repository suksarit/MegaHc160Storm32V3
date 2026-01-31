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
// PHASE 2 – PUBLIC API
// ============================================================================

// 👉 เข้า safe state (state transition เท่านั้น)
// ❗ ไม่ตัดฮาร์ดแวร์
void enterSafeState(FaultCode code);

// อ่าน fault ที่ถูก latch
FaultCode getActiveFault();

// 👉 ตัดฮาร์ดแวร์จริง (motor / blade / ignition)
// ❗ เรียกเฉพาะตอน exit loop หรือ escalation
void handleFaultImmediateCut();
