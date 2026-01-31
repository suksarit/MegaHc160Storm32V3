// ========================================================================================
// VoltageSensor.cpp
// ========================================================================================
#include "VoltageSensor.h"
#include "SystemConfig.h"

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

bool VoltageSensor::update(float &engineVolt) {

  if (!initialized || !adsPtr) {
    return false;
  }

  // ---------- OVERSAMPLE ----------
  int32_t sum = 0;
  for (uint8_t i = 0; i < 5; i++) {
    sum += adsPtr->readADC_SingleEnded(VOLTAGE_ADS_CHANNEL);
  }

  // ---------- CONVERT ----------
  float v = (sum / 5) * ADS_LSB_VOLT * VOLT_DIV_RATIO;

  // ---------- PLAUSIBILITY ----------
  if (v < 0.0f || v > VOLT_MAX_PLAUSIBLE) {
    return false;
  }

  // ---------- LPF ----------
  engineVolt += VOLT_LPF_ALPHA * (v - engineVolt);
  lastOk_ms = millis();

  return true;
}
