// ========================================================================================
// DriveStateMachine.cpp  
// ========================================================================================
#include "DriveStateMachine.h"
#include "DriveTarget.h"
#include "SystemTypes.h"
#include "SystemConfig.h"
#include "RuntimeContext.h"

// ========================================================================================
// GLOBAL CONTEXT
// ========================================================================================
extern RuntimeContext g_ctx;

// ========================================================================================
// INTERNAL STATE
// ========================================================================================
static uint32_t driveSoftStopStart_ms = 0;

// ================= WARN DERATE CONFIG =================
// ลดกำลังเมื่อ WARN (ไม่เปลี่ยน state)
static constexpr uint8_t WARN_DERATE_PERCENT = 70;   // 70%

// ========================================================================================
// IMPLEMENT
// ========================================================================================
void DriveStateMachine::update(uint32_t now) {

  static DriveState last = DriveState::IDLE;
  static uint32_t limpSafeStart_ms = 0;

  switch (g_ctx.driveState) {

    // --------------------------------------------------
    case DriveState::IDLE:
      g_ctx.targetL = 0;
      g_ctx.targetR = 0;
      g_ctx.curL    = 0;
      g_ctx.curR    = 0;

      limpSafeStart_ms = 0;
      driveSoftStopStart_ms = 0;

      if (g_ctx.systemState == SystemState::ACTIVE) {
        g_ctx.driveState = DriveState::RUN;
      }
      break;

    // --------------------------------------------------
    case DriveState::RUN:
      DriveTarget::update();

      // ---------- SAFETY HANDLING ----------
      if (g_ctx.driveSafety == SafetyState::EMERGENCY) {
        g_ctx.driveState = DriveState::SOFT_STOP;
        break;
      }

      if (g_ctx.driveSafety == SafetyState::LIMP) {
        g_ctx.driveState = DriveState::LIMP;
        limpSafeStart_ms = 0;
        break;
      }

      // ---------- WARN DERATE ----------
      if (g_ctx.driveSafety == SafetyState::WARN) {
        g_ctx.targetL = (g_ctx.targetL * WARN_DERATE_PERCENT) / 100;
        g_ctx.targetR = (g_ctx.targetR * WARN_DERATE_PERCENT) / 100;
      }
      break;

    // --------------------------------------------------
    case DriveState::LIMP:
      DriveTarget::update();

      // ลดกำลังแบบ hard สำหรับ LIMP
      g_ctx.targetL /= 2;
      g_ctx.targetR /= 2;

      if (g_ctx.driveSafety == SafetyState::EMERGENCY) {
        g_ctx.driveState = DriveState::SOFT_STOP;
        break;
      }

      // ฟื้นจาก LIMP → SAFE เท่านั้น
      if (g_ctx.driveSafety == SafetyState::SAFE) {
        if (limpSafeStart_ms == 0) {
          limpSafeStart_ms = now;
        } else if (now - limpSafeStart_ms >= LIMP_RECOVER_MS) {
          g_ctx.driveState = DriveState::RUN;
          limpSafeStart_ms = 0;
        }
      } else {
        limpSafeStart_ms = 0;
      }
      break;

    // --------------------------------------------------
    case DriveState::SOFT_STOP:
      g_ctx.targetL = 0;
      g_ctx.targetR = 0;

      if ((g_ctx.curL == 0 && g_ctx.curR == 0) ||
          (now - driveSoftStopStart_ms >= DRIVE_SOFT_STOP_TIMEOUT_MS)) {
        g_ctx.driveState = DriveState::LOCKED;
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
  if (g_ctx.driveState != last) {

    if (g_ctx.driveState == DriveState::SOFT_STOP) {
      driveSoftStopStart_ms = now;
    }

    last = g_ctx.driveState;
  }
}
