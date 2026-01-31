// ========================================================================================
// SystemTypes.h
// Central system-wide type definitions
// MUST be included by ALL .ino / .cpp that use system states
//
// PHASE 1 – PRODUCTION HARDENED
// ========================================================================================

#pragma once
#include <Arduino.h>

// ============================================================================
// SAFETY STATE
// Ordered by severity (SAFE → EMERGENCY)
// ============================================================================
enum class SafetyState : uint8_t {
  SAFE = 0,
  WARN,
  LIMP,
  EMERGENCY
};

// ============================================================================
// DRIVE EVENT
// Transient events for logging / diagnostics ONLY
// ⚠️ MUST NOT be used as control state machine
// ============================================================================
enum class DriveEvent : uint8_t {
  NONE = 0,

  // ---- traction / motion ----
  IMBALANCE,
  STUCK_LEFT,
  STUCK_RIGHT,
  WHEEL_LOCK,
  AUTO_REVERSE,

  // ---- current / torque ----
  CURRENT_WARN,
  TORQUE_LIMIT
};

// ============================================================================
// SYSTEM STATE MACHINE
// High-level system lifecycle
//
// INIT -> WAIT_NEUTRAL -> WAIT_BLADE_ARM -> ACTIVE
// FAULT is terminal (requires reboot / reset)
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
// Controls wheel motion only
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
// Blade must STOP on SafetyState::EMERGENCY
// ============================================================================
enum class BladeState : uint8_t {
  IDLE = 0,
  RUN,
  SOFT_STOP,
  LOCKED
};
