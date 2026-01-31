// ========================================================================================
// MotorOutput.cpp  
// ========================================================================================
#include "MotorOutput.h"
#include "RuntimeContext.h"
#include "SystemConfig.h"
#include "PinMap.h"
#include <Arduino.h>

// ================= RUNTIME CONTEXT =================
extern RuntimeContext g_ctx;

// HW safe hook (ยังเป็น function hook ตามเดิม)
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
  if (g_ctx.systemState == SystemState::FAULT ||
      g_ctx.driveState  == DriveState::LOCKED) {

    driveSafe();

    g_ctx.curL    = 0;
    g_ctx.curR    = 0;
    g_ctx.targetL = 0;
    g_ctx.targetR = 0;

    revBlockUntilL = 0;
    revBlockUntilR = 0;

    setMotorL(0);
    setMotorR(0);
    return;
  }

  // ---------- EMERGENCY ----------
  if (g_ctx.driveSafety == SafetyState::EMERGENCY) {
    g_ctx.targetL = 0;
    g_ctx.targetR = 0;
  }

  // ---------- REVERSE DEADTIME ----------
  bool wantRevL = (g_ctx.curL > 0 && g_ctx.targetL < 0) ||
                  (g_ctx.curL < 0 && g_ctx.targetL > 0);

  bool wantRevR = (g_ctx.curR > 0 && g_ctx.targetR < 0) ||
                  (g_ctx.curR < 0 && g_ctx.targetR > 0);

  if (wantRevL && revBlockUntilL == 0)
    revBlockUntilL = now + REVERSE_DEADTIME_MS;

  if (wantRevR && revBlockUntilR == 0)
    revBlockUntilR = now + REVERSE_DEADTIME_MS;

  if (revBlockUntilL && now < revBlockUntilL)
    g_ctx.targetL = 0;
  else if (revBlockUntilL && now >= revBlockUntilL)
    revBlockUntilL = 0;

  if (revBlockUntilR && now < revBlockUntilR)
    g_ctx.targetR = 0;
  else if (revBlockUntilR && now >= revBlockUntilR)
    revBlockUntilR = 0;

  // ---------- RAMP SELECT ----------
  int16_t step;
  if (g_ctx.targetL == 0 && g_ctx.targetR == 0)      step = 2;
  else if (g_ctx.driveState == DriveState::SOFT_STOP) step = 2;
  else if (g_ctx.driveState == DriveState::LIMP)      step = 4;
  else                                                 step = 5;

  // ---------- APPLY RAMP ----------
  g_ctx.curL = ramp(g_ctx.curL, g_ctx.targetL, step);
  g_ctx.curR = ramp(g_ctx.curR, g_ctx.targetR, step);

  // ---------- OUTPUT ----------
  setMotorL(g_ctx.curL);
  setMotorR(g_ctx.curR);
}
