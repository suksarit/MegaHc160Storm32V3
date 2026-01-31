// ========================================================================================
// VoltageSensor.cpp  (FIXED / HARDENED)
// ========================================================================================
#include "VoltageSensor.h"
#include "SystemConfig.h"

// ================= CONSTANTS =================
static constexpr uint8_t  VOLT_OVERSAMPLE_COUNT = 5;
static constexpr uint32_t VOLT_TIMEOUT_MS       = 200;

// ================= INTERNAL =================
static Adafruit_ADS1115 *adsPtr = nullptr;
static bool initialized = false;
static uint32_t lastOk_ms = 0;

// ================= IMPLEMENT =================
void VoltageSensor::begin(Adafruit_ADS1115 &ads) {

  adsPtr = &ads;
  initialized = true;
  lastOk_ms = 0;
}

bool VoltageSensor::update(float &engineVolt, uint32_t now) {

  if (!initialized || !adsPtr) {
    return false;
  }

  // ---------- OVERSAMPLE ----------
  int32_t sum = 0;
  for (uint8_t i = 0; i < VOLT_OVERSAMPLE_COUNT; i++) {
    sum += adsPtr->readADC_SingleEnded(VOLTAGE_ADS_CHANNEL);
  }

  // ---------- CONVERT ----------
  float v = (sum / VOLT_OVERSAMPLE_COUNT) * ADS_LSB_VOLT * VOLT_DIV_RATIO;

  // ---------- PLAUSIBILITY ----------
  if (v < 0.0f || v > VOLT_MAX_PLAUSIBLE) {
    // ignore spike, do not kill system
    if ((now - lastOk_ms) > VOLT_TIMEOUT_MS) {
      return false;
    }
    return true;
  }

  // ---------- LPF ----------
  engineVolt += VOLT_LPF_ALPHA * (v - engineVolt);
  lastOk_ms = now;

  return true;
}
