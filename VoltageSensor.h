// ========================================================================================
// VoltageSensor.h  (FIXED / MATCHED WITH .cpp)
// ========================================================================================
#pragma once

#include <Arduino.h>
#include <Adafruit_ADS1X15.h>

namespace VoltageSensor {

  // owner = .ino
  void begin(Adafruit_ADS1115 &ads);

  // อ่านแรงดันระบบ (engine / battery bus)
  // engineVolt : ค่าแรงดัน (LPF)
  // now        : เวลาปัจจุบันจาก millis()
  //
  // return true  = OK / usable
  // return false = sensor fault / stale / implausible
  bool update(float &engineVolt, uint32_t now);

}
