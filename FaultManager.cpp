// ========================================================================================
// FaultManager.cpp
// ========================================================================================
#include "FaultManager.h"

#include <EEPROM.h>
#include <Servo.h>

// ================= EXTERN FROM SYSTEM (.ino) =================
extern SystemState systemState;
extern DriveState  driveState;
extern BladeState  bladeState;
extern SafetyState driveSafety;

extern float   curA[4];
extern int16_t curL;
extern int16_t curR;
extern int16_t tempDriverL;
extern int16_t tempDriverR;

// ---------------- HW hooks ----------------
extern void driveSafe();
extern Servo bladeServo;

// ---------------- Critical outputs ----------------
extern uint8_t PIN_DRV_ENABLE;
extern uint8_t RELAY_IGNITION;
extern uint8_t RELAY_STARTER;
extern uint8_t PIN_BUZZER;
extern uint8_t RELAY_WARN;

// ============================================================================
// FAULT STATE
// ============================================================================
FaultCode activeFault = FaultCode::NONE;
bool faultLatched = false;

// ============================================================================
// LATCH FAULT (NO POLICY, SNAPSHOT ONLY)
// ============================================================================
void latchFault(FaultCode code) {

  if (faultLatched) return;

  activeFault  = code;
  faultLatched = true;

  // --------------------------------------------------
  // EEPROM FAULT HISTORY
  // --------------------------------------------------
  EEPROM.put(100, code);

#if DEBUG_SERIAL
  Serial.println(F("========== FAULT SNAPSHOT =========="));
  Serial.print(F("FaultCode="));
  Serial.println((uint8_t)code);

  Serial.print(F("SystemState="));
  Serial.println((uint8_t)systemState);
  Serial.print(F("DriveState="));
  Serial.println((uint8_t)driveState);
  Serial.print(F("BladeState="));
  Serial.println((uint8_t)bladeState);
  Serial.print(F("Safety="));
  Serial.println((uint8_t)driveSafety);

  Serial.print(F("CurA: "));
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(curA[i]);
    Serial.print(F(" "));
  }
  Serial.println();

  Serial.print(F("TempDriverL="));
  Serial.print(tempDriverL);
  Serial.print(F(" TempDriverR="));
  Serial.println(tempDriverR);

  Serial.print(F("PWM L/R="));
  Serial.print(curL);
  Serial.print(F("/"));
  Serial.println(curR);

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
  curL = 0;
  curR = 0;

  // -----------------------------
  // DRIVER ENABLE
  // -----------------------------
  digitalWrite(PIN_DRV_ENABLE, LOW);

  // -----------------------------
  // ENGINE / BLADE
  // -----------------------------
  bladeServo.writeMicroseconds(1000);
  digitalWrite(RELAY_IGNITION, LOW);
  digitalWrite(RELAY_STARTER,  LOW);

  // -----------------------------
  // ALERT / BUZZER
  // -----------------------------
  digitalWrite(PIN_BUZZER, HIGH);
  digitalWrite(RELAY_WARN, HIGH);
}
