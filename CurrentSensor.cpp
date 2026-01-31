// ========================================================================================
// CurrentSensor.cpp
// ========================================================================================
#include "CurrentSensor.h"
#include "SystemConfig.h"
#include <math.h>

// ================= EXTERN (owner = .ino) =================
extern float g_acsOffsetV[4];

// ================= CONSTANTS (HARDWARE) =================
static constexpr float ACS_SENS_V_PER_A = 0.04f;
static constexpr float ADS_LSB_V = 4.096f / 32768.0f;

// ================= INTERNAL =================
static Adafruit_ADS1115 *adsPtr = nullptr;
static float curA[CUR_CHANNELS] = {0};

static uint32_t lastActivity_ms = 0;

// ================= IMPLEMENT =================
void CurrentSensor::begin(Adafruit_ADS1115 &ads) {

  adsPtr = &ads;
  lastActivity_ms = 0;

  for (uint8_t i = 0; i < CUR_CHANNELS; i++) {
    curA[i] = 0.0f;
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
      return false;
    }

    // ---------- LPF ----------
    curA[i] += CUR_LPF_ALPHA * (a - curA[i]);

    // ---------- ACTIVITY CHECK ----------
    if (expectActivity) {
      if (fabs(curA[i] - a) > 0.05f) {
        lastActivity_ms = now;
      }
      if (now - lastActivity_ms > 150) {
        return false;
      }
    } else {
      lastActivity_ms = now;
    }
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
