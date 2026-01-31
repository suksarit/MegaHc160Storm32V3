// ========================================================================================
// TempSensor.cpp  (FIXED / HARDENED)
// ========================================================================================
#include "TempSensor.h"
#include "SystemConfig.h"
#include <math.h>

// ================= CONSTANTS =================
static constexpr float    TEMP_MIN_PLAUSIBLE   = -40.0f;
static constexpr float    TEMP_MAX_PLAUSIBLE   = 200.0f;
static constexpr float    TEMP_LPF_ALPHA       = 0.2f;
static constexpr uint32_t TEMP_TIMEOUT_MS      = 500;

// ================= INTERNAL =================
static Adafruit_MAX31865 *maxL = nullptr;
static Adafruit_MAX31865 *maxR = nullptr;
static bool initialized = false;

static float tempL_f = 0.0f;
static float tempR_f = 0.0f;

static uint32_t lastOkL_ms = 0;
static uint32_t lastOkR_ms = 0;

// ================= IMPLEMENT =================
void TempSensor::begin(Adafruit_MAX31865 &l,
                       Adafruit_MAX31865 &r) {

  maxL = &l;
  maxR = &r;
  initialized = true;

  tempL_f = 0.0f;
  tempR_f = 0.0f;
  lastOkL_ms = 0;
  lastOkR_ms = 0;
}

bool TempSensor::update(int16_t &tL, int16_t &tR, uint32_t now) {

  if (!initialized || !maxL || !maxR) {
    return false;
  }

  // ---------- FAULT CHECK ----------
  uint8_t faultL = maxL->readFault();
  uint8_t faultR = maxR->readFault();

  if (faultL) {
    maxL->clearFault();
  }
  if (faultR) {
    maxR->clearFault();
  }

  // ---------- READ TEMPERATURE ----------
  float TL = maxL->temperature(RTD_RNOMINAL, RTD_RREF);
  float TR = maxR->temperature(RTD_RNOMINAL, RTD_RREF);

  // ---------- PLAUSIBILITY ----------
  bool okL = (TL >= TEMP_MIN_PLAUSIBLE && TL <= TEMP_MAX_PLAUSIBLE);
  bool okR = (TR >= TEMP_MIN_PLAUSIBLE && TR <= TEMP_MAX_PLAUSIBLE);

  if (okL) {
    tempL_f += TEMP_LPF_ALPHA * (TL - tempL_f);
    lastOkL_ms = now;
  }

  if (okR) {
    tempR_f += TEMP_LPF_ALPHA * (TR - tempR_f);
    lastOkR_ms = now;
  }

  // ---------- TIMEOUT CHECK ----------
  if ((now - lastOkL_ms) > TEMP_TIMEOUT_MS ||
      (now - lastOkR_ms) > TEMP_TIMEOUT_MS) {
    return false;
  }

  tL = (int16_t)lroundf(tempL_f);
  tR = (int16_t)lroundf(tempR_f);

  return true;
}
