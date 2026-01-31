// ========================================================================================
// MegaHc160Storm32.ino  - RC Lawn Mower By TN MOWER
// CONTEXT-BASED / LINKER-SAFE / PINMAP-CORRECT
//
//   ✔ Task 1: Loop Timing Enforcement
//   ✔ Task 2: Sensor Warm-up & Plausibility
//   ✔ Task 3: RecoveryManager (AUTO / MANUAL / LOCKOUT)
//   ✔ Task 4: MANUAL confirm via IBUS
//   ✔ Task 5.5: Retry + Blacklist Fault
//   ✔ Task 5: Central Safe State
// ========================================================================================

#define DEBUG_SERIAL 1
#if DEBUG_SERIAL
  #define DBG(x)   Serial.print(x)
  #define DBGL(x)  Serial.println(x)
#else
  #define DBG(x)
  #define DBGL(x)
#endif

#define TEST_MODE 0

// ========================================================================================
// INCLUDES
// ========================================================================================
#include <Arduino.h>
#include <IBusBM.h>
#include <Servo.h>
#include <avr/wdt.h>
#include <Wire.h>
#include <EEPROM.h>
#include <SPI.h>

#include <Adafruit_ADS1X15.h>
#include <Adafruit_MAX31865.h>

#include "PinMap.h"
#include "SystemTypes.h"
#include "SystemConfig.h"
#include "RuntimeContext.h"
#include "SafetyManager.h"
#include "FaultManager.h"
#include "RecoveryManager.h"
#include "SDLogger.h"
#include "TempSensor.h"
#include "VoltageSensor.h"
#include "CurrentSensor.h"
#include "DriveStateMachine.h"
#include "MotorOutput.h"

// ========================================================================================
// GLOBAL RUNTIME CONTEXT
// ========================================================================================
RuntimeContext g_ctx = {
  SystemState::INIT,
  DriveState::IDLE,
  BladeState::IDLE,
  SafetyState::SAFE,
  DriveEvent::NONE,
  false,

  0, 0, 0, 0,

  {0, 0, 0, 0},
  0.0f,
  0, 0,

  0, 0, 0,

  false, false, false, false
};

// ========================================================================================
// DEVICES
// ========================================================================================
IBusBM ibus;
uint32_t lastIbusByte_ms = 0;
bool ibusCommLost = true;

uint32_t sensorWarmupStart_ms = 0;
bool     sensorWarmupDone     = false;

Adafruit_ADS1115 adsCur;
Adafruit_ADS1115 adsVolt;
Adafruit_MAX31865 maxL(PIN_MAX_CS_L);
Adafruit_MAX31865 maxR(PIN_MAX_CS_R);

Servo bladeServo;
RecoveryManager recovery;

// ========================================================================================
// FUNCTION PROTOTYPES (LINKER-SAFE)
// ========================================================================================
void updateComms(uint32_t now);
void updateSensors(uint32_t now);
bool sensorPlausible();
void driveSafe();

// ========================================================================================
// TASK 4: MANUAL CONFIRM VIA IBUS
// ========================================================================================
#define IBUS_CONFIRM_HOLD_MS 2500
#define IBUS_CONFIRM_TH      1100
uint32_t ibusConfirmStart_ms = 0;

static bool checkIbusManualConfirm(uint32_t now) {

  if (recovery.getMode() != RecoveryMode::MANUAL) {
    ibusConfirmStart_ms = 0;
    return false;
  }

  uint16_t ch1 = ibus.readChannel(0);
  uint16_t ch2 = ibus.readChannel(1);

  bool gesture = (ch1 < IBUS_CONFIRM_TH) &&
                 (ch2 < IBUS_CONFIRM_TH);

  if (gesture) {
    if (ibusConfirmStart_ms == 0) {
      ibusConfirmStart_ms = now;
    } else if (now - ibusConfirmStart_ms >= IBUS_CONFIRM_HOLD_MS) {
      ibusConfirmStart_ms = 0;
      return true;
    }
  } else {
    ibusConfirmStart_ms = 0;
  }

  return false;
}

// ========================================================================================
// PWM SAFE CUT
// ========================================================================================
void driveSafe() {
  OCR3A = 0; OCR3B = 0;
  OCR4A = 0; OCR4B = 0;
}

// ========================================================================================
// SENSOR PLAUSIBILITY
// ========================================================================================
bool sensorPlausible() {

  for (uint8_t i = 0; i < 4; i++) {
    if (g_ctx.curA[i] < CUR_MIN_PLAUSIBLE ||
        g_ctx.curA[i] > CUR_MAX_PLAUSIBLE) {
      return false;
    }
  }

  if (g_ctx.engineVolt < 0.0f ||
      g_ctx.engineVolt > VOLT_MAX_PLAUSIBLE) {
    return false;
  }

  if (g_ctx.tempDriverL < -20 || g_ctx.tempDriverL > 150) return false;
  if (g_ctx.tempDriverR < -20 || g_ctx.tempDriverR > 150) return false;

  return true;
}

// ========================================================================================
// COMMS
// ========================================================================================
void updateComms(uint32_t now) {

  while (Serial1.available()) {
    ibus.loop();
    lastIbusByte_ms = now;
    ibusCommLost = false;
  }

  if (now - lastIbusByte_ms > IBUS_TIMEOUT_MS) {
    ibusCommLost = true;
    g_ctx.driveSafety = SafetyState::EMERGENCY;
  }

  g_ctx.wdCommsOK = !ibusCommLost;
}

// ========================================================================================
// SENSORS
// ========================================================================================
void updateSensors(uint32_t now) {

#if TEST_MODE
  for (uint8_t i = 0; i < 4; i++) g_ctx.curA[i] = 5.0f;
  g_ctx.tempDriverL = 45;
  g_ctx.tempDriverR = 47;
  g_ctx.engineVolt  = 26.0f;
  g_ctx.wdSensorOK  = true;
  return;
#endif

  if (digitalRead(PIN_CUR_TRIP) == LOW) {
    enterSafeState(FaultCode::OVER_CURRENT);
    return;
  }

  if (!CurrentSensor::update(now, g_ctx.curL, g_ctx.curR)) {
    enterSafeState(FaultCode::CUR_SENSOR_FAULT);
    return;
  }

  for (uint8_t i = 0; i < 4; i++) {
    g_ctx.curA[i] = CurrentSensor::get(i);
  }

  if (!TempSensor::update(g_ctx.tempDriverL, g_ctx.tempDriverR, now)) {
    enterSafeState(FaultCode::TEMP_SENSOR_FAULT);
    return;
  }

  if (!VoltageSensor::update(g_ctx.engineVolt, now)) {
    enterSafeState(FaultCode::VOLT_SENSOR_FAULT);
    return;
  }

  g_ctx.wdSensorOK = true;
}

// ========================================================================================
// SETUP
// ========================================================================================
void setup() {

  Serial.begin(115200);
  Serial1.begin(115200);
  ibus.begin(Serial1);

  sensorWarmupStart_ms = millis();
  recovery.begin();

  Wire.begin();
  SPI.begin();

  TempSensor::begin(maxL, maxR);
  CurrentSensor::begin(adsCur);
  VoltageSensor::begin(adsVolt);

  MotorOutput::begin();
  SDLogger::begin();

  wdt_enable(WDTO_1S);
  g_ctx.systemState = SystemState::ACTIVE;
}

// ========================================================================================
// LOOP
// ========================================================================================
void loop() {

  uint32_t now = millis();
  wdt_reset();

  updateComms(now);
  updateSensors(now);

  // ---- sensor warm-up ----
  if (!sensorWarmupDone) {
    if (now - sensorWarmupStart_ms < SENSOR_WARMUP_MS) return;
    sensorWarmupDone = true;
  }

  if (!sensorPlausible()) {
    enterSafeState(FaultCode::CUR_SENSOR_FAULT);
  }

  // ---- SAFE / RECOVERY ----
  if (g_ctx.driveSafety == SafetyState::SAFE) {

    if (recovery.getMode() == RecoveryMode::NONE) {
      recovery.onFault(getActiveFault());
    }

    if (checkIbusManualConfirm(now)) {
      recovery.confirmManualRecovery();
      DBG("[RECOVERY] Manual confirm via IBUS\n");
    }

    recovery.update(now);

    if (!recovery.isRecoveryAllowed()) {
      driveSafe();
      return;
    }
  }

  DriveStateMachine::update(now);
  MotorOutput::apply(now);
}
