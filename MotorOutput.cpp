// ========================================================================================
// MotorOutput.cpp
// ========================================================================================
#include "MotorOutput.h"
#include "SystemTypes.h"   
#include "SystemConfig.h"

// ================= EXTERN =================
extern int16_t targetL, targetR;
extern int16_t curL, curR;

extern DriveState driveState;
extern SafetyState driveSafety;
extern SystemState systemState;

extern uint32_t revBlockUntilL;
extern uint32_t revBlockUntilR;
extern const uint16_t REVERSE_DEADTIME_MS;

// HW hooks (อยู่ใน .ino)
extern void setPWM_L(uint16_t v);
extern void setPWM_R(uint16_t v);
extern void driveSafe();

extern uint8_t DIR_L1, DIR_L2, DIR_R1, DIR_R2;

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

// ================= IMPLEMENT =================
void MotorOutput::apply(uint32_t now) {

  if (systemState == SystemState::FAULT || driveState == DriveState::LOCKED) {
    driveSafe();
    curL = curR = targetL = targetR = 0;
    revBlockUntilL = revBlockUntilR = 0;
    return;
  }

  if (driveSafety == SafetyState::EMERGENCY) {
    targetL = targetR = 0;
  }

  bool wantRevL = (curL > 0 && targetL < 0) || (curL < 0 && targetL > 0);
  bool wantRevR = (curR > 0 && targetR < 0) || (curR < 0 && targetR > 0);

  if (wantRevL && revBlockUntilL == 0) revBlockUntilL = now + REVERSE_DEADTIME_MS;
  if (wantRevR && revBlockUntilR == 0) revBlockUntilR = now + REVERSE_DEADTIME_MS;

  if (revBlockUntilL && now < revBlockUntilL) targetL = 0;
  else if (revBlockUntilL && now >= revBlockUntilL) revBlockUntilL = 0;

  if (revBlockUntilR && now < revBlockUntilR) targetR = 0;
  else if (revBlockUntilR && now >= revBlockUntilR) revBlockUntilR = 0;

  int16_t step;
  if (targetL == 0 && targetR == 0) step = 2;
  else if (driveState == DriveState::SOFT_STOP) step = 2;
  else if (driveState == DriveState::LIMP) step = 4;
  else step = 5;

  curL = ramp(curL, targetL, step);
  curR = ramp(curR, targetR, step);

  digitalWrite(DIR_L1, curL > 0);
  digitalWrite(DIR_L2, curL < 0);
  digitalWrite(DIR_R1, curR > 0);
  digitalWrite(DIR_R2, curR < 0);

  setPWM_L(abs(curL));
  setPWM_R(abs(curR));
}
