// ========================================================================================
// RuntimeContext.h  (PHASE 1 / PRODUCTION-GRADE)
// ========================================================================================
#pragma once

#include <Arduino.h>
#include "SystemTypes.h"

// ========================================================================================
// RUNTIME CONTEXT
// Single Source of Truth for ALL runtime state
// Owner: MegaHc160Storm32.ino
//
// ⚠️ NOTE:
// - Must NOT be written from ISR
// - Accessed only from main loop / modules
// ========================================================================================
struct RuntimeContext {

  // ------------------------------------------------------------------
  // SYSTEM STATES
  // ------------------------------------------------------------------
  SystemState systemState;
  DriveState  driveState;
  BladeState  bladeState;
  SafetyState driveSafety;
  DriveEvent  lastDriveEvent;
  bool        faultLatched;

  // ------------------------------------------------------------------
  // DRIVE VALUES
  // ------------------------------------------------------------------
  int16_t targetL;
  int16_t targetR;
  int16_t curL;
  int16_t curR;

  // ------------------------------------------------------------------
  // SENSOR VALUES
  // ------------------------------------------------------------------
  float   curA[4];
  float   engineVolt;
  int16_t tempDriverL;
  int16_t tempDriverR;

  // ------------------------------------------------------------------
  // HEALTH / DIAGNOSTIC
  // ------------------------------------------------------------------
  uint32_t loopCount;
  uint32_t lastLoop_us;
  uint32_t maxLoop_us;

  // ------------------------------------------------------------------
  // WATCHDOG FLAGS (uint8_t = atomic & explicit)
  // ------------------------------------------------------------------
  uint8_t wdCommsOK;
  uint8_t wdSensorOK;
  uint8_t wdDriveOK;
  uint8_t wdBladeOK;
};

// ========================================================================================
// GLOBAL CONTEXT INSTANCE (DECLARATION ONLY)
// ========================================================================================
extern RuntimeContext g_ctx;

// Guard against uncontrolled growth
static_assert(sizeof(RuntimeContext) < 128,
              "RuntimeContext too large – consider refactor");
