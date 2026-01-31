// ========================================================================================
// RuntimeContext.h
// ========================================================================================
#pragma once
#include <Arduino.h>
#include "SystemTypes.h"

// ================= RUNTIME CONTEXT =================
struct RuntimeContext {

  // ---------------- SYSTEM ----------------
  SystemState systemState;
  DriveState  driveState;
  BladeState  bladeState;
  SafetyState driveSafety;
  DriveEvent  lastDriveEvent;
  bool        faultLatched;

  // ---------------- DRIVE ----------------
  int16_t targetL;
  int16_t targetR;
  int16_t curL;
  int16_t curR;

  // ---------------- SENSORS ----------------
  float   curA[4];
  float   engineVolt;
  int16_t tempDriverL;
  int16_t tempDriverR;

  // ---------------- WATCHDOG ----------------
  volatile bool wdCommsOK;
  volatile bool wdSensorOK;
  volatile bool wdDriveOK;
  volatile bool wdBladeOK;
};

// single global instance (ONE owner)
extern RuntimeContext g_ctx;
