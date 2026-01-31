// ========================================================================================
// MotorOutput.cpp  (FIX: OWN reverse-deadtime state)
// ========================================================================================
#include "MotorOutput.h"
#include "SystemTypes.h"
#include "SystemConfig.h"
#include "PinMap.h"

// ================= EXTERN (logic owner) =================
extern int16_t targetL, targetR;
extern int16_t curL, curR;

extern DriveState  driveState;
extern SafetyState driveSafety;
extern SystemState systemState;

// HW safe hook
extern void driveSafe();

// ================= INTERNAL STATE (OWNED HERE) =================
static uint32_t revBlockUntilL = 0;
static uint32_t revBlockUntilR = 0;

// ================= UTIL =================
static int16_t ramp(int16_t c, int16_t t, int16_t s) {
  if (c < t) {
    c += s;
    if (c > t) c = t;
  } else if (c > t) {
    c -= s;
    if (c < t) c = t;
  }
  return c;
}

// ================= INTERNAL MOTOR IO =================
static inline void setMotorL(int16_t v) {
  digitalWrite(PIN_MOTOR_L_DIR, v > 0);
  digitalWrite(PIN_MOTOR_L_EN,  v != 0);
  analogWrite (PIN_MOTOR_L_PWM, abs(v));
}

static inline void setMotorR(int16_t v) {
  digitalWrite(PIN_MOTOR_R_DIR, v > 0);
  digitalWrite(PIN_MOTOR_R_EN,  v != 0);
  analogWrite (PIN_MOTOR_R_PWM, abs(v));
}

// ================= IMPLEMENT =================
void MotorOutput::begin() {

  pinMode(PIN_MOTOR_L_PWM, OUTPUT);
  pinMode(PIN_MOTOR_L_DIR, OUTPUT);
  pinMode(PIN_MOTOR_L_EN,  OUTPUT);

  pinMode(PIN_MOTOR_R_PWM, OUTPUT);
  pinMode(PIN_MOTOR_R_DIR, OUTPUT);
  pinMode(PIN_MOTOR_R_EN,  OUTPUT);

  setMotorL(0);
  setMotorR(0);

  revBlockUntilL = 0;
  revBlockUntilR = 0;
}

void MotorOutput::apply(uint32_t now) {

  // ---------- HARD FAULT ----------
  if (systemState == SystemState::FAULT ||
      driveState  == DriveState::LOCKED) {

    driveSafe();
    curL = curR = targetL = targetR = 0;
    revBlockUntilL = revBlockUntilR = 0;
    setMotorL(0);
    setMotorR(0);
    return;
  }

  // ---------- EMERGENCY ----------
  if (driveSafety == SafetyState::EMERGENCY) {
    targetL = targetR = 0;
  }

  // ---------- REVERSE DEADTIME ----------
  bool wantRevL = (curL > 0 && targetL < 0) ||
                  (curL < 0 && targetL > 0);
  bool wantRevR = (curR > 0 && targetR < 0) ||
                  (curR < 0 && targetR > 0);

  if (wantRevL && revBlockUntilL == 0)
    revBlockUntilL = now + REVERSE_DEADTIME_MS;

  if (wantRevR && revBlockUntilR == 0)
    revBlockUntilR = now + REVERSE_DEADTIME_MS;

  if (revBlockUntilL && now < revBlockUntilL) targetL = 0;
  else if (revBlockUntilL && now >= revBlockUntilL) revBlockUntilL = 0;

  if (revBlockUntilR && now < revBlockUntilR) targetR = 0;
  else if (revBlockUntilR && now >= revBlockUntilR) revBlockUntilR = 0;

  // ---------- RAMP SELECT ----------
  int16_t step;
  if (targetL == 0 && targetR == 0) step = 2;
  else if (driveState == DriveState::SOFT_STOP) step = 2;
  else if (driveState == DriveState::LIMP)      step = 4;
  else                                          step = 5;

  // ---------- APPLY RAMP ----------
  curL = ramp(curL, targetL, step);
  curR = ramp(curR, targetR, step);

  // ---------- OUTPUT ----------
  setMotorL(curL);
  setMotorR(curR);
}
