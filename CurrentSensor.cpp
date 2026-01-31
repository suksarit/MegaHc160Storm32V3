// ========================================================================================
// CurrentSensor.cpp  (FIXED / HARDENED)
// ========================================================================================
#include "CurrentSensor.h"
#include "SystemConfig.h"
#include <math.h>

// ================= CONSTANTS (HARDWARE) =================
static constexpr float ACS_SENS_V_PER_A = 0.04f;
static constexpr float ADS_LSB_V        = 4.096f / 32768.0f;

// ================= CONSTANTS (LOGIC) =================
static constexpr uint32_t CUR_ACTIVITY_TIMEOUT_MS = 150;
static constexpr float    CUR_ACTIVITY_DELTA_A    = 0.05f;

// ================= INTERNAL =================
static Adafruit_ADS1115 *adsPtr = nullptr;
static float curA[CUR_CHANNELS] = {0};

// activity tracking PER CHANNEL (FIX)
static uint32_t lastActivity_ms[CUR_CHANNELS] = {0};
static float g_acsOffsetV[4] = {2.5f, 2.5f, 2.5f, 2.5f};

void CurrentSensor::setOffset(uint8_t ch, float v) {
  if (ch < 4) g_acsOffsetV[ch] = v;
}

// ================= IMPLEMENT =================
void CurrentSensor::begin(Adafruit_ADS1115 &ads) {

  adsPtr = &ads;

  for (uint8_t i = 0; i < CUR_CHANNELS; i++) {
    curA[i] = 0.0f;
    lastActivity_ms[i] = 0;
  }
}

bool CurrentSensor::update(uint32_t now,
                           int16_t curL,
                           int16_t curR) {

  if (!adsPtr) return false;

  bool expectActivity = (abs(curL) > 200 || abs(curR) > 200);

  for (uint8_t i = 0; i < CUR_CHANNELS; i++) {

    // ---------- OVERSAMPLE ----------
    int32_t sum = 0;
    for (uint8_t n = 0; n < 4; n++) {
      sum += adsPtr->readADC_SingleEnded(i);
    }
    int32_t raw = sum / 4;

    // ---------- CONVERT ----------
    float v = raw * ADS_LSB_V;
    float a = (v - g_acsOffsetV[i]) / ACS_SENS_V_PER_A;

    // ---------- PLAUSIBILITY ----------
    if (a < CUR_MIN_PLAUSIBLE || a > CUR_MAX_PLAUSIBLE) {
      // reject this channel only
      continue;
    }

    // ---------- ACTIVITY CHECK ----------
    if (expectActivity) {
      if (fabs(a - curA[i]) > CUR_ACTIVITY_DELTA_A) {
        lastActivity_ms[i] = now;
      }
      if ((now - lastActivity_ms[i]) > CUR_ACTIVITY_TIMEOUT_MS) {
        // no activity on this channel, ignore but do not kill system
        continue;
      }
    } else {
      lastActivity_ms[i] = now;
    }

    // ---------- LPF ----------
    curA[i] += CUR_LPF_ALPHA * (a - curA[i]);
  }

  return true;
}

float CurrentSensor::get(uint8_t idx) {
  return (idx < CUR_CHANNELS) ? curA[idx] : 0.0f;
}

float CurrentSensor::left() {
  return curA[0] + curA[1];
}

float CurrentSensor::right() {
  return curA[2] + curA[3];
}
