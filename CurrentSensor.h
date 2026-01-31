// ========================================================================================
// CurrentSensor.h
// ========================================================================================
#pragma once

#include <Arduino.h>
#include <Adafruit_ADS1X15.h>

// ================= CONFIG =================
#define CUR_CHANNELS 4

// ================= API =================
namespace CurrentSensor {
   void setOffset(uint8_t ch, float v);

  // ต้องส่ง ADS1115 จากภายนอก (owner = .ino)
  void begin(Adafruit_ADS1115 &ads);

  // อ่านกระแสทั้งหมด + plausibility + activity check
  // return true  = OK
  // return false = sensor fault
  bool update(uint32_t now,
              int16_t curL,
              int16_t curR);

  float get(uint8_t idx);   // 0..3
  float left();             // L1 + L2
  float right();            // R1 + R2
}
