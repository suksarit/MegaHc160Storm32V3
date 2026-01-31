// ========================================================================================
// FaultManager.cpp  (PHASE 2 – SAFE STATE ENTRY)
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

// ---------------- HW hooks ----------------
extern void driveSafe();
extern Servo bladeServo;

// ============================================================================
// INTERNAL FAULT STATE
// ============================================================================
static FaultCode activeFault = FaultCode::NONE;
static bool hardCutDone = false;   // idempotent guard

// ============================================================================
// READ ACTIVE FAULT
// ============================================================================
FaultCode getActiveFault() {
  return activeFault;
}

// ============================================================================
// ENTER SAFE STATE  (PHASE 2 CORE)
// ============================================================================
void enterSafeState(FaultCode code) {

  // already in fault → do nothing
  if (g_ctx.faultLatched) return;

  activeFault        = code;
  g_ctx.faultLatched = true;

  // -----------------------------
  // STATE TRANSITION (LOGICAL)
  // -----------------------------
  g_ctx.systemState = SystemState::FAULT;
  g_ctx.driveState  = DriveState::LOCKED;
  g_ctx.bladeState  = BladeState::LOCKED;
  g_ctx.driveSafety = SafetyState::EMERGENCY;

  // clear drive intent
  g_ctx.targetL = 0;
  g_ctx.targetR = 0;

  // -----------------------------
  // PERSIST FAULT
  // -----------------------------
  EEPROM.put(100, code);

#if DEBUG_SERIAL
  Serial.println(F("========== ENTER SAFE STATE =========="));
  Serial.print(F("FaultCode="));
  Serial.println((uint8_t)code);
  Serial.println(F("System locked. Await recovery/reset."));
  Serial.println(F("===================================="));
#endif
}

// ============================================================================
// IMMEDIATE HARD CUT (LAST LINE OF DEFENSE)
// ============================================================================
void handleFaultImmediateCut() {

  if (hardCutDone) return;
  hardCutDone = true;

#if DEBUG_SERIAL
  Serial.println(F("[FAULT] HARD CUT"));
#endif

  // -----------------------------
  // MOTOR PWM
  // -----------------------------
  driveSafe();
  g_ctx.curL = 0;
  g_ctx.curR = 0;

  // -----------------------------
  // DRIVER ENABLE
  // -----------------------------
  digitalWrite(PIN_DRIVER_ENABLE, LOW);

  // -----------------------------
  // ENGINE / BLADE
  // -----------------------------
  bladeServo.writeMicroseconds(1000);
  digitalWrite(PIN_RELAY_IGNITION, LOW);
  digitalWrite(PIN_RELAY_STARTER,  LOW);

  // -----------------------------
  // ALERT
  // -----------------------------
  digitalWrite(PIN_BUZZER, HIGH);
  digitalWrite(PIN_RELAY_WARN, HIGH);
}
