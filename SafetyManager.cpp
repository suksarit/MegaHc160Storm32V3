// ========================================================================================
// SafetyManager.cpp
// ========================================================================================
#include "SafetyManager.h"
#include "SystemConfig.h"

// ============================================================================
// SINGLE SAFETY DECISION FUNCTION
// ============================================================================
SafetyState decideSafety(void) {

  // --------------------------------------------------
  // HARD FAULT ALWAYS WINS
  // --------------------------------------------------
  if (faultLatched) {
    return SafetyState::EMERGENCY;
  }

  // --------------------------------------------------
  // CURRENT CHECK (MAX OF ALL CHANNELS)
  // --------------------------------------------------
  float curMax = curA[0];
  for (uint8_t i = 1; i < 4; i++) {
    if (curA[i] > curMax) curMax = curA[i];
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
  if (tempDriverL >= TEMP_LIMP_C ||
      tempDriverR >= TEMP_LIMP_C) {
    return SafetyState::EMERGENCY;
  }

  if (tempDriverL >= TEMP_WARN_C ||
      tempDriverR >= TEMP_WARN_C) {
    return SafetyState::WARN;
  }

  // --------------------------------------------------
  // SAFE
  // --------------------------------------------------
  return SafetyState::SAFE;
}
