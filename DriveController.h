//  DriveController.h

#pragma once
#include <Arduino.h>
#include "SafetyManager.h"

// ================================
// DRIVE CONTROLLER
// ================================
// หน้าที่:
// - คำนวณ targetL / targetR จาก IBUS
// - state machine ด้าน drive
// - ramp / deadtime / soft stop
// - ไม่ยุ่ง sensor โดยตรง
// ================================

class DriveController {
public:
  static void begin();

  static void update(uint32_t now);

  static int16_t getTargetL();
  static int16_t getTargetR();
  static int16_t getCurL();
  static int16_t getCurR();

  static void forceStop(uint32_t now);

private:
  static void updateTarget();
  static void applyOutput(uint32_t now);
  static int16_t ramp(int16_t c, int16_t t, int16_t step);

  static int16_t targetL;
  static int16_t targetR;
  static int16_t curL;
  static int16_t curR;

  static uint32_t softStopStart_ms;
};
