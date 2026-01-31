// ========================================================================================
// SystemTypes.h
// Central system-wide type definitions
// MUST be included by ALL .ino / .cpp that use system states
// ========================================================================================

#pragma once
#include <Arduino.h>

// ============================================================================
// SAFETY STATE
// ============================================================================
enum class SafetyState : uint8_t {
  SAFE = 0,
  WARN,
  LIMP,
  EMERGENCY
};

// ============================================================================
// DRIVE EVENT (LOG / DEBUG / SD)
// ============================================================================
enum class DriveEvent : uint8_t {
  NONE = 0,

  // ---- imbalance / traction ----
  IMBALANCE,
  STUCK_LEFT,
  STUCK_RIGHT,
  WHEEL_LOCK,
  AUTO_REVERSE,

  // ---- current / torque ----
  CURRENT_WARN,     // กระแสสูงผิดปกติ แต่ยังไม่จำกัดแรงบิด
  TORQUE_LIMIT      // กำลังถูกลด torque อยู่ (soft limit)
};

// ============================================================================
// SYSTEM STATE MACHINE
// ============================================================================
enum class SystemState : uint8_t {
  INIT = 0,
  WAIT_NEUTRAL,
  WAIT_BLADE_ARM,
  ACTIVE,
  FAULT
};

// ============================================================================
// DRIVE STATE MACHINE
// ============================================================================
enum class DriveState : uint8_t {
  IDLE = 0,
  RUN,
  LIMP,
  SOFT_STOP,
  LOCKED
};

// ============================================================================
// BLADE STATE MACHINE
// ============================================================================
enum class BladeState : uint8_t {
  IDLE = 0,
  RUN,
  SOFT_STOP,
  LOCKED
};
