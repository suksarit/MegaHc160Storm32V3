// ========================================================================================
// SafetyManager.cpp  (CONTEXT-BASED / LINKER-SAFE)
// ========================================================================================
#include "SafetyManager.h"
#include "SystemConfig.h"
#include "RuntimeContext.h"

// ========================================================================================
// GLOBAL CONTEXT
// ========================================================================================
extern RuntimeContext g_ctx;

// ============================================================================
// SINGLE SAFETY DECISION FUNCTION
// ============================================================================
SafetyState decideSafety(void) {

  // --------------------------------------------------
  // HARD FAULT ALWAYS WINS
  // --------------------------------------------------
  if (g_ctx.faultLatched) {
    return SafetyState::EMERGENCY;
  }

  // --------------------------------------------------
  // CURRENT CHECK (MAX OF ALL CHANNELS)
  // --------------------------------------------------
  float curMax = g_ctx.curA[0];
  for (uint8_t i = 1; i < 4; i++) {
    if (g_ctx.curA[i] > curMax) {
      curMax = g_ctx.curA[i];
    }
  }

  // ---------- EMERGENCY ----------
  if (curMax >= CUR_LIMP_A) {
    return SafetyState::EMERGENCY;
  }

  // ---------- WARN ----------
  if (curMax >= CUR_WARN_A) {
    return SafetyState::WARN;
  }

  // --------------------------------------------------
  // TEMPERATURE CHECK
  // --------------------------------------------------
  if (g_ctx.tempDriverL >= TEMP_LIMP_C ||
      g_ctx.tempDriverR >= TEMP_LIMP_C) {
    return SafetyState::EMERGENCY;
  }

  if (g_ctx.tempDriverL >= TEMP_WARN_C ||
      g_ctx.tempDriverR >= TEMP_WARN_C) {
    return SafetyState::WARN;
  }

  // --------------------------------------------------
  // SAFE
  // --------------------------------------------------
  return SafetyState::SAFE;
}
