// ========================================================================================
// MegaHc160Storm32.ino  - RC Lawn Mower By TN MOWER
// CONTEXT-BASED / LINKER-SAFE / PINMAP-CORRECT
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
  // system
  SystemState::INIT,
  DriveState::IDLE,
  BladeState::IDLE,
  SafetyState::SAFE,
  DriveEvent::NONE,
  false,

  // drive
  0, 0, 0, 0,

  // sensors
  {0, 0, 0, 0},
  0.0f,
  0, 0,

  // watchdog
  false, false, false, false
};

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

// ========================================================================================
// CURRENT SENSOR OFFSET (ACS OWNERSHIP)
// ========================================================================================
float g_acsOffsetV[4] = { 2.50f, 2.50f, 2.50f, 2.50f };

// ========================================================================================
// ENGINE
// ========================================================================================
Servo bladeServo;

// ========================================================================================
// PROTOTYPES
// ========================================================================================
void updateComms(uint32_t now);
void updateSensors(uint32_t now);

// ========================================================================================
// PWM INIT (TIMER3 / TIMER4 @15kHz)
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

// ========================================================================================
// HARD CUT (LOW-LEVEL MOTOR ONLY)
// ========================================================================================
void driveSafe() {
  OCR3A = 0; OCR3B = 0;
  OCR4A = 0; OCR4B = 0;
}

// ========================================================================================
// CURRENT SPIKE DETECTOR
// ========================================================================================
bool checkCurrentSpike(uint32_t now) {

  static uint32_t spikeStart_ms = 0;

  float curMax = max(
    max(g_ctx.curA[0], g_ctx.curA[1]),
    max(g_ctx.curA[2], g_ctx.curA[3])
  );

  if (curMax >= CUR_SPIKE_A) {
    if (spikeStart_ms == 0) spikeStart_ms = now;
    if (now - spikeStart_ms >= 4) {
      g_ctx.lastDriveEvent = DriveEvent::WHEEL_LOCK;
      return true;
    }
  } else {
    spikeStart_ms = 0;
  }
  return false;
}

// ========================================================================================
// TORQUE LIMIT (CURRENT-BASED DERATE)
// ========================================================================================
void applyTorqueLimit() {

  static float scaleL = 1.0f;
  static float scaleR = 1.0f;

  float curLeft  = g_ctx.curA[0] + g_ctx.curA[1];
  float curRight = g_ctx.curA[2] + g_ctx.curA[3];

  if (curLeft > TORQUE_LIMIT_A) {
    scaleL -= (curLeft - TORQUE_LIMIT_A) * TORQUE_REDUCE_GAIN;
  } else if (curLeft < TORQUE_RECOVER_A) {
    scaleL += TORQUE_RECOVER_GAIN;
  }

  if (curRight > TORQUE_LIMIT_A) {
    scaleR -= (curRight - TORQUE_LIMIT_A) * TORQUE_REDUCE_GAIN;
  } else if (curRight < TORQUE_RECOVER_A) {
    scaleR += TORQUE_RECOVER_GAIN;
  }

  scaleL = constrain(scaleL, 0.0f, 1.0f);
  scaleR = constrain(scaleR, 0.0f, 1.0f);

  g_ctx.curL = (int16_t)(g_ctx.curL * scaleL);
  g_ctx.curR = (int16_t)(g_ctx.curR * scaleR);

  if (curLeft > TORQUE_HARD_CUT_A || curRight > TORQUE_HARD_CUT_A) {
    g_ctx.lastDriveEvent = DriveEvent::WHEEL_LOCK;
    latchFault(FaultCode::OVER_CURRENT);
  }
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
    latchFault(FaultCode::OVER_CURRENT);
    return;
  }

  if (!CurrentSensor::update(now, g_ctx.curL, g_ctx.curR)) {
    latchFault(FaultCode::CUR_SENSOR_FAULT);
    return;
  }

  for (uint8_t i = 0; i < 4; i++) {
    g_ctx.curA[i] = CurrentSensor::get(i);
  }

  if (!TempSensor::update(g_ctx.tempDriverL, g_ctx.tempDriverR, now)) {
    latchFault(FaultCode::TEMP_SENSOR_FAULT);
    return;
  }

  if (!VoltageSensor::update(g_ctx.engineVolt, now)) {
    latchFault(FaultCode::VOLT_SENSOR_FAULT);
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

  pinMode(PIN_DRIVER_ENABLE, OUTPUT);
  pinMode(PIN_CUR_TRIP, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_RELAY_WARN, OUTPUT);

  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_RELAY_WARN, LOW);
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

  wdt_reset();
  uint32_t now = millis();

  updateComms(now);
  updateSensors(now);

  if (checkCurrentSpike(now)) {
    latchFault(FaultCode::OVER_CURRENT);
  }

  g_ctx.driveSafety = decideSafety();
  DriveStateMachine::update(now);
  applyTorqueLimit();

  static uint32_t warnBeep_ms = 0;
  static bool buzOn = false;
  static bool lastWarn = false;

  if (g_ctx.driveSafety == SafetyState::WARN) {

    if (now - warnBeep_ms >= 200) {
      warnBeep_ms = now;
      buzOn = !buzOn;
    }

    digitalWrite(PIN_BUZZER, buzOn ? HIGH : LOW);
    digitalWrite(PIN_RELAY_WARN, HIGH);

    if (!lastWarn) {
      SDLogger::logWarn(
        now,
        g_ctx.curA,
        g_ctx.tempDriverL,
        g_ctx.tempDriverR,
        g_ctx.driveSafety
      );
    }

  } else {
    digitalWrite(PIN_BUZZER, LOW);
    digitalWrite(PIN_RELAY_WARN, LOW);
  }

  lastWarn = (g_ctx.driveSafety == SafetyState::WARN);

  MotorOutput::apply(now);

  // ---------- FAULT ----------
  if (g_ctx.faultLatched) {
    driveSafe();
    digitalWrite(PIN_DRIVER_ENABLE, LOW);   // ✅ FIX: cut driver
    SDLogger::flush();
    return;
  }

  digitalWrite(PIN_DRIVER_ENABLE, HIGH);

  SDLogger::log(now);
  SDLogger::flush();

  g_ctx.wdDriveOK = true;
  g_ctx.wdBladeOK = true;
}
