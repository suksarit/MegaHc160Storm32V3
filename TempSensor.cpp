// ========================================================================================
// TempSensor.cpp
// ========================================================================================
#include "TempSensor.h"
#include "SystemConfig.h"

// ================= INTERNAL =================
static Adafruit_MAX31865 *maxL = nullptr;
static Adafruit_MAX31865 *maxR = nullptr;
static bool initialized = false;

// ================= IMPLEMENT =================
void TempSensor::begin(Adafruit_MAX31865 &l,
                       Adafruit_MAX31865 &r) {

  maxL = &l;
  maxR = &r;
  initialized = true;
}

bool TempSensor::update(int16_t &tL, int16_t &tR) {

  if (!initialized || !maxL || !maxR) {
    return false;
  }

  // ---------- FAULT CHECK ----------
  uint8_t faultL = maxL->readFault();
  uint8_t faultR = maxR->readFault();

  if (faultL || faultR) {
    maxL->clearFault();
    maxR->clearFault();
    return false;
  }

  // ---------- READ TEMPERATURE ----------
  float TL = maxL->temperature(RTD_RNOMINAL, RTD_RREF);
  float TR = maxR->temperature(RTD_RNOMINAL, RTD_RREF);

  // ---------- PLAUSIBILITY ----------
  if (TL < -40.0f || TL > 200.0f) return false;
  if (TR < -40.0f || TR > 200.0f) return false;

  tL = (int16_t)TL;
  tR = (int16_t)TR;
  return true;
}
