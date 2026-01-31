// ========================================================================================
// MegaHc160Storm32.ino  - RC Lawn Mower By TN MOWER
// CONTEXT-BASED / LINKER-SAFE / PINMAP-CORRECT
// PHASE 1 COMPLETE (PRODUCTION-GRADE)
//
//   ✔ Task 1: Loop Timing Enforcement
//   ✔ Task 2: Sensor Warm-up & Plausibility
//   ✔ Task 3: Logic Watchdog
//   ✔ Task 5: Central Safe State (NO latchFault DIRECT CALL)
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
#include "SDLogger.h"
#include "TempSensor.h"
#include "VoltageSensor.h"
#include "CurrentSensor.h"
#include "DriveStateMachine.h"
#include "MotorOutput.h"

// ========================================================================================
// GLOBAL RUNTIME CONTEXT (SINGLE OWNER)
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
// SENSOR WARM-UP
// ========================================================================================
uint32_t sensorWarmupStart_ms = 0;
bool     sensorWarmupDone     = false;

// ========================================================================================
// LOGIC WATCHDOG
// ========================================================================================
uint32_t logicWdtLastKick_ms = 0;

// ========================================================================================
// DEVICES
// ========================================================================================
IBusBM ibus;
uint32_t lastIbusByte_ms = 0;
bool ibusCommLost = true;

Adafruit_ADS1115 adsCur;
Adafruit_ADS1115 adsVolt;
Adafruit_MAX31865 maxL(PIN_MAX_CS_L);
Adafruit_MAX31865 maxR(PIN_MAX_CS_R);

float g_acsOffsetV[4] = { 2.50f, 2.50f, 2.50f, 2.50f };
Servo bladeServo;

// ========================================================================================
// PROTOTYPES
// ========================================================================================
void updateComms(uint32_t now);
void updateSensors(uint32_t now);
void applyTorqueLimit();   // <<< FIX: add prototype

// ========================================================================================
// PWM INIT
// ========================================================================================
void initMotorPWM() {

  TCCR3A = 0; TCCR3B = 0;
  TCCR3A = (1 << COM3A1) | (1 << COM3B1) | (1 << WGM31);
  TCCR3B = (1 << WGM33)  | (1 << WGM32)  | (1 << CS30);
  ICR3   = PWM_TOP;
  OCR3A  = 0;
  OCR3B  = 0;

  TCCR4A = 0; TCCR4B = 0;
  TCCR4A = (1 << COM4A1) | (1 << COM4B1) | (1 << WGM41);
  TCCR4B = (1 << WGM43)  | (1 << WGM42)  | (1 << CS40);
  ICR4   = PWM_TOP;
  OCR4A  = 0;
  OCR4B  = 0;
}

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
// TORQUE LIMIT (CURRENT-BASED DERATE)
// ========================================================================================
void applyTorqueLimit() {

  static float scaleL = 1.0f;
  static float scaleR = 1.0f;

  float curLeft  = g_ctx.curA[0] + g_ctx.curA[1];
  float curRight = g_ctx.curA[2] + g_ctx.curA[3];

  // -------- LEFT --------
  if (curLeft > TORQUE_LIMIT_A) {
    scaleL -= (curLeft - TORQUE_LIMIT_A) * TORQUE_REDUCE_GAIN;
    g_ctx.lastDriveEvent = DriveEvent::TORQUE_LIMIT;
  } else if (curLeft < TORQUE_RECOVER_A) {
    scaleL += TORQUE_RECOVER_GAIN;
  }

  // -------- RIGHT --------
  if (curRight > TORQUE_LIMIT_A) {
    scaleR -= (curRight - TORQUE_LIMIT_A) * TORQUE_REDUCE_GAIN;
    g_ctx.lastDriveEvent = DriveEvent::TORQUE_LIMIT;
  } else if (curRight < TORQUE_RECOVER_A) {
    scaleR += TORQUE_RECOVER_GAIN;
  }

  scaleL = constrain(scaleL, 0.0f, 1.0f);
  scaleR = constrain(scaleR, 0.0f, 1.0f);

  g_ctx.curL = (int16_t)(g_ctx.curL * scaleL);
  g_ctx.curR = (int16_t)(g_ctx.curR * scaleR);

  // -------- HARD CUT GUARD --------
  if (curLeft > TORQUE_HARD_CUT_A || curRight > TORQUE_HARD_CUT_A) {
    enterSafeState(FaultCode::OVER_CURRENT);
  }
}

// ========================================================================================
// SETUP
// ========================================================================================
void setup() {

  Serial.begin(115200);
  Serial1.begin(115200);
  ibus.begin(Serial1);

  sensorWarmupStart_ms = millis();
  logicWdtLastKick_ms  = millis();

  pinMode(PIN_DRIVER_ENABLE, OUTPUT);
  pinMode(PIN_CUR_TRIP, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_RELAY_WARN, OUTPUT);

  digitalWrite(PIN_DRIVER_ENABLE, LOW);

  initMotorPWM();
  driveSafe();

  bladeServo.attach(PIN_SERVO_ENGINE);
  bladeServo.writeMicroseconds(1000);

  Wire.begin();
  SPI.begin();

  maxL.begin(MAX31865_3WIRE);
  maxR.begin(MAX31865_3WIRE);
  TempSensor::begin(maxL, maxR);

  adsCur.begin(0x48);
  adsCur.setGain(GAIN_ONE);
  CurrentSensor::begin(adsCur);

  adsVolt.begin(0x49);
  adsVolt.setGain(GAIN_ONE);
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

  uint32_t loopStart_us = micros();
  wdt_reset();
  uint32_t now = millis();

  // ---- reset watchdog flags ----
  g_ctx.wdCommsOK  = false;
  g_ctx.wdSensorOK = false;
  g_ctx.wdDriveOK  = false;
  g_ctx.wdBladeOK  = false;

  updateComms(now);
  updateSensors(now);

  // ---- sensor warm-up ----
  if (!sensorWarmupDone) {
    if (now - sensorWarmupStart_ms >= SENSOR_WARMUP_MS) {
      sensorWarmupDone = true;
    } else {
      goto LOOP_END;
    }
  }

  if (!sensorPlausible()) {
    enterSafeState(FaultCode::CUR_SENSOR_FAULT);
  }

  g_ctx.driveSafety = decideSafety();
  DriveStateMachine::update(now);
  applyTorqueLimit();
  MotorOutput::apply(now);

  // ---- logic watchdog ----
  bool logicOK =
    g_ctx.wdCommsOK &&
    g_ctx.wdSensorOK &&
    g_ctx.wdDriveOK &&
    g_ctx.wdBladeOK;

  if (logicOK) {
    logicWdtLastKick_ms = now;
  } else if (now - logicWdtLastKick_ms > LOGIC_WDT_MS) {
    enterSafeState(FaultCode::LOGIC_WATCHDOG);
  }

LOOP_END:

  uint32_t loopTime_us = micros() - loopStart_us;

  g_ctx.loopCount++;
  g_ctx.lastLoop_us = loopTime_us;
  if (loopTime_us > g_ctx.maxLoop_us) g_ctx.maxLoop_us = loopTime_us;

  if (loopTime_us > (uint32_t)BUDGET_LOOP_MS * 1000UL) {
    enterSafeState(FaultCode::LOOP_OVERRUN);
  }

  g_ctx.wdDriveOK = true;
  g_ctx.wdBladeOK = true;
}
