// ========================================================================================
// TempSensor.h
// ========================================================================================
#pragma once

#include <Arduino.h>
#include <Adafruit_MAX31865.h>

namespace TempSensor {

  // owner = .ino (ต้อง begin MAX31865 ก่อน)
  void begin(Adafruit_MAX31865 &left,
             Adafruit_MAX31865 &right);

  // อ่านอุณหภูมิคนขับซ้าย/ขวา
  // return true  = OK
  // return false = sensor fault / plausibility fail
  bool update(int16_t &tL, int16_t &tR);
}
