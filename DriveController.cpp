#include "DriveController.h"
#include <IBusBM.h>

// ================================
// EXTERN (จาก .ino)
// ================================
extern IBusBM ibus;
extern SystemState systemState;
extern DriveState driveState;
extern SafetyState driveSafety;

// PWM TOP ต้องตรงกับของจริง
constexpr int16_t PWM_TOP = 1067;

// ================================
// STATIC STORAGE
// ================================
int16_t DriveController::targetL = 0;
int16_t DriveController::targetR = 0;
int16_t DriveController::curL = 0;
int16_t DriveController::curR = 0;

uint32_t DriveController::softStopStart_ms = 0;

// ================================
// LOCAL UTIL
// ================================
static inline bool neutral(uint16_t v) {
  return (v > 1450 && v < 1550);
}

// ================================
// PUBLIC
// ================================
void DriveController::begin() {
  targetL = targetR = 0;
  curL = curR = 0;
  softStopStart_ms = 0;
}

void DriveController::update(uint32_t now) {

  switch (driveState) {

    case DriveState::IDLE:
      targetL = targetR = 0;
      curL = curR = 0;
      if (systemState == SystemState::ACTIVE) {
        driveState = DriveState::RUN;
      }
      break;

    case DriveState::RUN:
      updateTarget();
      if (driveSafety == SafetyState::EMERGENCY) {
        driveState = DriveState::SOFT_STOP;
        softStopStart_ms = now;
      }
      break;

    case DriveState::SOFT_STOP:
      targetL = targetR = 0;
      if (now - softStopStart_ms > 1500) {
        driveState = DriveState::LOCKED;
      }
      break;

    case DriveState::LOCKED:
      targetL = targetR = 0;
      curL = curR = 0;
      break;
  }

  applyOutput(now);
}

int16_t DriveController::getTargetL() { return targetL; }
int16_t DriveController::getTargetR() { return targetR; }
int16_t DriveController::getCurL() { return curL; }
int16_t DriveController::getCurR() { return curR; }

// ================================
// PRIVATE
// ================================
void DriveController::updateTarget() {

  if (systemState != SystemState::ACTIVE || driveSafety == SafetyState::EMERGENCY) {
    targetL = targetR = 0;
    return;
  }

  auto mapAxis = [](int16_t v) -> int16_t {
    constexpr int16_t IN_MIN = 1000;
    constexpr int16_t IN_MAX = 2000;
    constexpr int16_t DB_MIN = 1450;
    constexpr int16_t DB_MAX = 1550;

    if (v >= DB_MIN && v <= DB_MAX) return 0;
    if (v < DB_MIN) return map(v, IN_MIN, DB_MIN, -PWM_TOP, 0);
    return map(v, DB_MAX, IN_MAX, 0, PWM_TOP);
  };

  int16_t thr = mapAxis(ibus.readChannel(2));  // throttle
  int16_t str = mapAxis(ibus.readChannel(1));  // steer

  targetL = thr + str;
  targetR = thr - str;

  targetL = constrain(targetL, -PWM_TOP, PWM_TOP);
  targetR = constrain(targetR, -PWM_TOP, PWM_TOP);
}

void DriveController::applyOutput(uint32_t /*now*/) {

  int16_t step = 5;

  if (driveState == DriveState::SOFT_STOP) step = 2;
  if (driveState == DriveState::LOCKED) step = 0;

  curL = ramp(curL, targetL, step);
  curR = ramp(curR, targetR, step);

  // OUTPUT จริง ถูกทำใน .ino (setPWM / DIR)
}

int16_t DriveController::ramp(int16_t c, int16_t t, int16_t step) {
  if (c < t) {
    c += step;
    if (c > t) c = t;
  } else if (c > t) {
    c -= step;
    if (c < t) c = t;
  }
  return c;
}
