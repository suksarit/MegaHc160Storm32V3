// ========================================================================================
// FaultManager.cpp  (CONTEXT-BASED / PINMAP-CORRECT)
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

// ---------------- HW hooks (INTENTIONAL EXTERN) ----------------
extern void driveSafe();
extern Servo bladeServo;

// ============================================================================
// FAULT STATE (OWNED HERE)
// ============================================================================
static FaultCode activeFault = FaultCode::NONE;

// ============================================================================
// LATCH FAULT (NO POLICY, SNAPSHOT ONLY)
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
// IMMEDIATE HARD CUT (LAST LINE OF DEFENSE)
// ============================================================================
void handleFaultImmediateCut() {

#if DEBUG_SERIAL
  static bool printed = false;
  if (!printed) {
    Serial.println(F("[FAULT] IMMEDIATE HARD CUT"));
    printed = true;
  }
#endif

  // -----------------------------
  // MOTOR DRIVE (PWM + DIR)
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
  // ALERT / BUZZER
  // -----------------------------
  digitalWrite(PIN_BUZZER, HIGH);
  digitalWrite(PIN_RELAY_WARN, HIGH);
}
