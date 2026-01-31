// ========================================================================================
// FaultManager.cpp  (PHASE 1 / TASK 5 – CENTRAL SAFE STATE)
// ========================================================================================
#include "FaultManager.h"

#include <EEPROM.h>
#include <Servo.h>

#include "RuntimeContext.h"
#include "PinMap.h"

// ========================================================================================
// GLOBAL CONTEXT
// ========================================================================================
extern RuntimeContext g_ctx;

// ---------------- HW hooks (LOW LEVEL) ----------------
extern void driveSafe();
extern Servo bladeServo;

// ============================================================================
// FAULT STATE (OWNED HERE ONLY)
// ============================================================================
static FaultCode activeFault = FaultCode::NONE;
static bool safeStateEntered = false;   // idempotent guard

// ============================================================================
// READ ACTIVE FAULT
// ============================================================================
FaultCode getActiveFault() {
  return activeFault;
}

// ============================================================================
// LATCH FAULT (SNAPSHOT ONLY – NO ACTION)
// ============================================================================
void latchFault(FaultCode code) {

  if (g_ctx.faultLatched) return;

  activeFault        = code;
  g_ctx.faultLatched = true;

  EEPROM.put(100, code);

#if DEBUG_SERIAL
  Serial.println(F("========== FAULT SNAPSHOT =========="));
  Serial.print(F("FaultCode="));
  Serial.println((uint8_t)code);

  Serial.print(F("SystemState="));
  Serial.println((uint8_t)g_ctx.systemState);
  Serial.print(F("DriveState="));
  Serial.println((uint8_t)g_ctx.driveState);
  Serial.print(F("BladeState="));
  Serial.println((uint8_t)g_ctx.bladeState);
  Serial.print(F("Safety="));
  Serial.println((uint8_t)g_ctx.driveSafety);

  Serial.print(F("CurA: "));
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(g_ctx.curA[i]);
    Serial.print(F(" "));
  }
  Serial.println();

  Serial.print(F("TempDriverL="));
  Serial.print(g_ctx.tempDriverL);
  Serial.print(F(" TempDriverR="));
  Serial.println(g_ctx.tempDriverR);

  Serial.print(F("PWM L/R="));
  Serial.print(g_ctx.curL);
  Serial.print(F("/"));
  Serial.println(g_ctx.curR);

  Serial.println(F("===================================="));
#endif
}

// ============================================================================
// CENTRAL SAFE STATE (SINGLE SOURCE OF KILL)
// ============================================================================
void enterSafeState(FaultCode code) {

  if (safeStateEntered) return;   // idempotent
  safeStateEntered = true;

  // ---- latch fault if not already ----
  latchFault(code);

  // ---- system state ----
  g_ctx.systemState = SystemState::FAULT;
  g_ctx.driveState  = DriveState::LOCKED;
  g_ctx.bladeState  = BladeState::LOCKED;

#if DEBUG_SERIAL
  Serial.print(F("[SAFE STATE] ENTER, fault="));
  Serial.println((uint8_t)code);
#endif

  // -----------------------------
  // MOTOR DRIVE (PWM OFF)
  // -----------------------------
  driveSafe();
  g_ctx.curL = 0;
  g_ctx.curR = 0;
  g_ctx.targetL = 0;
  g_ctx.targetR = 0;

  // -----------------------------
  // DRIVER ENABLE
  // -----------------------------
  digitalWrite(PIN_DRIVER_ENABLE, LOW);

  // -----------------------------
  // ENGINE / BLADE
  // -----------------------------
  bladeServo.writeMicroseconds(1000);     // idle throttle
  digitalWrite(PIN_RELAY_IGNITION, LOW);
  digitalWrite(PIN_RELAY_STARTER,  LOW);

  // -----------------------------
  // ALERT
  // -----------------------------
  digitalWrite(PIN_BUZZER, HIGH);
  digitalWrite(PIN_RELAY_WARN, HIGH);
}

// ============================================================================
// IMMEDIATE HARD CUT (LOW LEVEL, NO POLICY)
// ============================================================================
void handleFaultImmediateCut() {

  // ใช้เฉพาะกรณีสุดท้ายจริง ๆ
#if DEBUG_SERIAL
  Serial.println(F("[FAULT] IMMEDIATE HARD CUT"));
#endif

  driveSafe();
  digitalWrite(PIN_DRIVER_ENABLE, LOW);

  bladeServo.writeMicroseconds(1000);
  digitalWrite(PIN_RELAY_IGNITION, LOW);
  digitalWrite(PIN_RELAY_STARTER,  LOW);

  digitalWrite(PIN_BUZZER, HIGH);
  digitalWrite(PIN_RELAY_WARN, HIGH);
}
