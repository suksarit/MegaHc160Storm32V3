// ========================================================================================
// VoltageSensor.h
// ========================================================================================
#pragma once

#include <Arduino.h>
#include <Adafruit_ADS1X15.h>

namespace VoltageSensor {

  // owner = .ino
  void begin(Adafruit_ADS1115 &ads);

  // อ่านแรงดันระบบ (engine / battery bus)
  // return true  = OK
  // return false = sensor fault / implausible
  bool update(float &engineVolt);
}
