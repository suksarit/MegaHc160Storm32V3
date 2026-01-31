// ========================================================================================
// FaultManager.h  (PHASE 1 / TASK 5.1 – CENTRAL SAFE STATE)
// ========================================================================================
#pragma once

#include <Arduino.h>
#include "SystemTypes.h"

// ============================================================================
// FAULT CODE (SINGLE SOURCE OF TRUTH)
// ============================================================================
enum class FaultCode : uint8_t {
  NONE = 0,

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
// PUBLIC API – FAULT CONTROL
// ============================================================================

/**
 * @brief Latch fault code ครั้งเดียว (ไม่ overwrite)
 *        - บันทึก fault
 *        - set g_ctx.faultLatched = true
 *        - ❌ ไม่ตัด hardware
 */
void latchFault(FaultCode code);

/**
 * @brief อ่าน fault ที่ถูก latch ไว้
 *        ใช้สำหรับ logging / SD / debug
 */
FaultCode getActiveFault();

/**
 * @brief เข้าสู่ Safe State แบบ deterministic (CENTRAL KILL SWITCH)
 *
 * Behavior:
 *  - latch fault (ถ้ายังไม่ latch)
 *  - systemState = FAULT
 *  - ตัด motor PWM
 *  - disable driver
 *  - stop blade
 *  - cut ignition + starter
 *  - activate buzzer / warn relay
 *
 * ❗ เป็นฟังก์ชันเดียวที่ “ฆ่าของจริง”
 * ❗ เรียกจาก loop / watchdog / state machine เท่านั้น
 */
void enterSafeState(FaultCode code);

/**
 * @brief Immediate hard cut (LOW LEVEL, NO POLICY)
 *        ใช้เฉพาะกรณีสุดท้ายจริง ๆ
 *        (ยังคงไว้เพื่อ compatibility)
 */
void handleFaultImmediateCut();
