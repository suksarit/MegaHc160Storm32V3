// ========================================================================================
// TempSensor.h  (FIXED / MATCHED WITH .cpp)
// ========================================================================================
#pragma once

#include <Arduino.h>
#include <Adafruit_MAX31865.h>

namespace TempSensor {

  // owner = .ino (ต้อง begin MAX31865 ก่อน)
  void begin(Adafruit_MAX31865 &left,
             Adafruit_MAX31865 &right);

  // อ่านอุณหภูมิไดรเวอร์ซ้าย/ขวา
  // tL, tR : อุณหภูมิ (°C, filtered)
  // now    : millis()
  //
  // return true  = OK / usable
  // return false = sensor stale / hard fault
  bool update(int16_t &tL, int16_t &tR, uint32_t now);
}
