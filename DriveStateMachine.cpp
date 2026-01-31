// ========================================================================================
// DriveStateMachine.cpp
// ========================================================================================
#include "DriveStateMachine.h"
#include "DriveTarget.h"
#include "SystemTypes.h"
#include "SystemConfig.h"

// ================= EXTERN =================
extern DriveState  driveState;
extern SystemState systemState;
extern SafetyState driveSafety;

extern int16_t targetL;
extern int16_t targetR;
extern int16_t curL;
extern int16_t curR;

static uint32_t driveSoftStopStart_ms = 0;
// ================= WARN DERATE CONFIG =================
// ลดกำลังเมื่อ WARN (ไม่เปลี่ยน state)
static constexpr uint8_t WARN_DERATE_PERCENT = 70;   // 70%

// ================= IMPLEMENT =================
void DriveStateMachine::update(uint32_t now) {

  static DriveState last = DriveState::IDLE;
  static uint32_t limpSafeStart_ms = 0;

  switch (driveState) {

    // --------------------------------------------------
    case DriveState::IDLE:
      targetL = 0;
      targetR = 0;
      curL    = 0;
      curR    = 0;

      limpSafeStart_ms = 0;
      driveSoftStopStart_ms = 0;

      if (systemState == SystemState::ACTIVE) {
        driveState = DriveState::RUN;
      }
      break;

    // --------------------------------------------------
    case DriveState::RUN:
      DriveTarget::update();

      // ---------- SAFETY HANDLING ----------
      if (driveSafety == SafetyState::EMERGENCY) {
        driveState = DriveState::SOFT_STOP;
        break;
      }

      if (driveSafety == SafetyState::LIMP) {
        driveState = DriveState::LIMP;
        limpSafeStart_ms = 0;
        break;
      }

      // ---------- WARN DERATE ----------
      if (driveSafety == SafetyState::WARN) {
        targetL = (targetL * WARN_DERATE_PERCENT) / 100;
        targetR = (targetR * WARN_DERATE_PERCENT) / 100;
      }

      break;

    // --------------------------------------------------
    case DriveState::LIMP:
      DriveTarget::update();

      // ลดกำลังแบบ hard สำหรับ LIMP
      targetL /= 2;
      targetR /= 2;

      if (driveSafety == SafetyState::EMERGENCY) {
        driveState = DriveState::SOFT_STOP;
        break;
      }

      // ฟื้นจาก LIMP → SAFE เท่านั้น
      if (driveSafety == SafetyState::SAFE) {
        if (limpSafeStart_ms == 0) {
          limpSafeStart_ms = now;
        } else if (now - limpSafeStart_ms >= LIMP_RECOVER_MS) {
          driveState = DriveState::RUN;
          limpSafeStart_ms = 0;
        }
      } else {
        limpSafeStart_ms = 0;
      }
      break;

    // --------------------------------------------------
    case DriveState::SOFT_STOP:
      targetL = 0;
      targetR = 0;

      if ((curL == 0 && curR == 0) ||
          (now - driveSoftStopStart_ms >= DRIVE_SOFT_STOP_TIMEOUT_MS)) {
        driveState = DriveState::LOCKED;
      }
      break;

    // --------------------------------------------------
    case DriveState::LOCKED:
      // latch state – รอ reset จากระบบหลัก
      break;
  }

  // --------------------------------------------------
  // STATE TRANSITION HOOK
  // --------------------------------------------------
  if (driveState != last) {

    if (driveState == DriveState::SOFT_STOP) {
      driveSoftStopStart_ms = now;
    }

    last = driveState;
  }
}
