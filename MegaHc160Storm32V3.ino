// ========================================================================================
// MegaHc160Storm32.ino  - RC Lawn Mower By TN MOWER
// CLEAN / MODULAR / FIELD-SAFE VERSION (PWM 15kHz TIMER3/4)
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

#include "SystemTypes.h"
#include "SystemConfig.h"
#include "SafetyManager.h"
#include "FaultManager.h"
#include "SDLogger.h"
#include "TempSensor.h"
#include "VoltageSensor.h"
#include "CurrentSensor.h"
#include "DriveStateMachine.h"

// ========================================================================================
// HARDWARE PIN MAP
// ========================================================================================
constexpr uint8_t PIN_DRV_ENABLE = 33;

constexpr uint8_t DIR_L1 = 5;  // OC3A
constexpr uint8_t DIR_L2 = 2;  // OC3B
constexpr uint8_t DIR_R1 = 6;  // OC4A
constexpr uint8_t DIR_R2 = 7;  // OC4B

constexpr uint8_t SERVO_ENGINE_PIN = 9;
constexpr uint8_t MAX_CS_L = 49;
constexpr uint8_t MAX_CS_R = 48;
constexpr uint8_t PIN_BUZZER = 30;
constexpr uint8_t RELAY_WARN = 31;
constexpr uint8_t PIN_CUR_TRIP = 32;

// ========================================================================================
// SYSTEM STATES (OWNED HERE)
// ========================================================================================
SystemState systemState = SystemState::INIT;
DriveState  driveState  = DriveState::IDLE;
BladeState  bladeState  = BladeState::IDLE;
SafetyState driveSafety = SafetyState::SAFE;

// ========================================================================================
// DRIVE EVENT
// ========================================================================================
DriveEvent lastDriveEvent = DriveEvent::NONE;

// ========================================================================================
// DEVICES
// ========================================================================================
IBusBM ibus;
uint32_t lastIbusByte_ms = 0;
bool ibusCommLost = true;

Adafruit_ADS1115 adsCur;
Adafruit_ADS1115 adsVolt;
Adafruit_MAX31865 maxL(MAX_CS_L);
Adafruit_MAX31865 maxR(MAX_CS_R);

// ========================================================================================
// CURRENT SENSOR OFFSET (ACS OFFSET OWNERSHIP)
// ========================================================================================
// ใช้โดย CurrentSensor.cpp (extern)
float g_acsOffsetV[4] = {
  2.50f,   // Left motor A
  2.50f,   // Left motor B
  2.50f,   // Right motor A
  2.50f    // Right motor B
};

// ========================================================================================
// DRIVE VALUES
// ========================================================================================
int16_t targetL = 0;
int16_t targetR = 0;
int16_t curL = 0;
int16_t curR = 0;

// ========================================================================================
// ENGINE
// ========================================================================================
Servo bladeServo;
float engineVolt = 0.0f;

// ========================================================================================
// SENSOR RUNTIME
// ========================================================================================
float   curA[4] = {0};
int16_t tempDriverL = 0;
int16_t tempDriverR = 0;

// ========================================================================================
// TORQUE / WARN STATUS
// ========================================================================================
bool torqueLimited = false;

// ========================================================================================
// WATCHDOG FLAGS
// ========================================================================================
volatile bool wdCommsOK  = false;
volatile bool wdSensorOK = false;
volatile bool wdDriveOK  = false;
volatile bool wdBladeOK  = false;

// ========================================================================================
// FUNCTION PROTOTYPES (REQUIRED IN .ino)
// ========================================================================================
void updateComms(uint32_t now);
void updateSensors(uint32_t now);

// ========================================================================================
// PWM INIT
// ========================================================================================
void initMotorPWM() {

  TCCR3A = 0; TCCR3B = 0;
  TCCR3A = (1 << COM3A1) | (1 << COM3B1) | (1 << WGM31);
  TCCR3B = (1 << WGM33)  | (1 << WGM32)  | (1 << CS30);
  ICR3 = PWM_TOP;
  OCR3A = 0;
  OCR3B = 0;

  TCCR4A = 0; TCCR4B = 0;
  TCCR4A = (1 << COM4A1) | (1 << COM4B1) | (1 << WGM41);
  TCCR4B = (1 << WGM43)  | (1 << WGM42)  | (1 << CS40);
  ICR4 = PWM_TOP;
  OCR4A = 0;
  OCR4B = 0;
}

// ========================================================================================
// HARD CUT
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
    max(curA[0], curA[1]),
    max(curA[2], curA[3])
  );

  if (curMax >= CUR_SPIKE_A) {

    if (spikeStart_ms == 0) spikeStart_ms = now;

    if (now - spikeStart_ms >= 4) {
      lastDriveEvent = DriveEvent::WHEEL_LOCK;
      return true;
    }

  } else {
    spikeStart_ms = 0;
  }

  return false;
}

// ========================================================================================
// TORQUE LIMIT (CURRENT-BASED, SOFT → WARN)
// ========================================================================================
void applyTorqueLimit() {

  static float scaleL = 1.0f;
  static float scaleR = 1.0f;

  torqueLimited = false;

  float curLeft  = curA[0] + curA[1];
  float curRight = curA[2] + curA[3];

  if (curLeft > TORQUE_LIMIT_A) {
    scaleL -= (curLeft - TORQUE_LIMIT_A) * TORQUE_REDUCE_GAIN;
    torqueLimited = true;
  } else if (curLeft < TORQUE_RECOVER_A) {
    scaleL += TORQUE_RECOVER_GAIN;
  }

  if (curRight > TORQUE_LIMIT_A) {
    scaleR -= (curRight - TORQUE_LIMIT_A) * TORQUE_REDUCE_GAIN;
    torqueLimited = true;
  } else if (curRight < TORQUE_RECOVER_A) {
    scaleR += TORQUE_RECOVER_GAIN;
  }

  scaleL = constrain(scaleL, 0.0f, 1.0f);
  scaleR = constrain(scaleR, 0.0f, 1.0f);

  curL = (int16_t)(curL * scaleL);
  curR = (int16_t)(curR * scaleR);

  if (curLeft > TORQUE_HARD_CUT_A ||
      curRight > TORQUE_HARD_CUT_A) {

    lastDriveEvent = DriveEvent::WHEEL_LOCK;
    latchFault(FaultCode::OVER_CURRENT);
  }
}

// ========================================================================================
// APPLY DRIVE OUTPUT (PWM + DEADTIME)
// ========================================================================================
void applyDriveOutput() {

  static int8_t lastDirL = 0;
  static int8_t lastDirR = 0;

  if (driveSafety == SafetyState::EMERGENCY ||
      driveState  == DriveState::LOCKED) {

    driveSafe();
    lastDirL = lastDirR = 0;
    curL = curR = 0;
    return;
  }

  int8_t dirL = (curL > 0) - (curL < 0);
  int8_t dirR = (curR > 0) - (curR < 0);

  uint16_t dutyL = constrain(abs(curL), 0, PWM_TOP);
  uint16_t dutyR = constrain(abs(curR), 0, PWM_TOP);

  if (dirL != lastDirL && lastDirL != 0) {
    driveSafe();
    delayMicroseconds(PWM_DEADTIME_US);
  }
  OCR3A = (dirL > 0) ? dutyL : 0;
  OCR3B = (dirL < 0) ? dutyL : 0;
  lastDirL = dirL;

  if (dirR != lastDirR && lastDirR != 0) {
    driveSafe();
    delayMicroseconds(PWM_DEADTIME_US);
  }
  OCR4A = (dirR > 0) ? dutyR : 0;
  OCR4B = (dirR < 0) ? dutyR : 0;
  lastDirR = dirR;
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
    driveSafety = SafetyState::EMERGENCY;
  }

  wdCommsOK = !ibusCommLost;
}

// ========================================================================================
// SENSORS
// ========================================================================================
void updateSensors(uint32_t now) {

#if TEST_MODE
  for (uint8_t i = 0; i < 4; i++) curA[i] = 5.0f;
  tempDriverL = 45;
  tempDriverR = 47;
  engineVolt  = 26.0f;
  wdSensorOK = true;
  return;
#endif

  if (digitalRead(PIN_CUR_TRIP) == LOW) {
    latchFault(FaultCode::OVER_CURRENT);
    return;
  }

  if (!CurrentSensor::update(now, curL, curR)) {
    latchFault(FaultCode::CUR_SENSOR_FAULT);
    return;
  }

  for (uint8_t i = 0; i < 4; i++) {
    curA[i] = CurrentSensor::get(i);
  }

  if (!TempSensor::update(tempDriverL, tempDriverR)) {
    latchFault(FaultCode::TEMP_SENSOR_FAULT);
    return;
  }

  if (!VoltageSensor::update(engineVolt)) {
    latchFault(FaultCode::VOLT_SENSOR_FAULT);
    return;
  }

  wdSensorOK = true;
}

// ========================================================================================
// SETUP
// ========================================================================================
void setup() {

  Serial.begin(115200);
  Serial1.begin(115200);

  ibus.begin(Serial1);

  pinMode(PIN_DRV_ENABLE, OUTPUT);

  pinMode(DIR_L1, OUTPUT);
  pinMode(DIR_L2, OUTPUT);
  pinMode(DIR_R1, OUTPUT);
  pinMode(DIR_R2, OUTPUT);

  pinMode(PIN_CUR_TRIP, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(RELAY_WARN, OUTPUT);

  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(RELAY_WARN, LOW);
  digitalWrite(PIN_DRV_ENABLE, LOW);

  // PWM
  initMotorPWM();
  driveSafe();

  // Servo
  bladeServo.attach(SERVO_ENGINE_PIN);
  bladeServo.writeMicroseconds(1000);

  // Buses
  Wire.begin();
  SPI.begin();

  // Temperature
  maxL.begin(MAX31865_3WIRE);
  maxR.begin(MAX31865_3WIRE);
  TempSensor::begin(maxL, maxR);

  // Current
  adsCur.begin(0x48);
  adsCur.setGain(GAIN_ONE);
  CurrentSensor::begin(adsCur);

  // Voltage
  adsVolt.begin(0x49);
  adsVolt.setGain(GAIN_ONE);
  VoltageSensor::begin(adsVolt);

  // SD
  SDLogger::begin();

  // Watchdog
  wdt_enable(WDTO_1S);

  systemState = SystemState::ACTIVE;
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

  driveSafety = decideSafety();
  DriveStateMachine::update(now);

  applyTorqueLimit();

  if (torqueLimited && driveSafety == SafetyState::SAFE) {
    driveSafety = SafetyState::WARN;
  }

  static uint32_t warnBeep_ms = 0;
  static bool buzOn = false;
  static bool lastWarn = false;

  if (driveSafety == SafetyState::WARN) {

    if (now - warnBeep_ms >= 200) {
      warnBeep_ms = now;
      buzOn = !buzOn;
    }

    digitalWrite(PIN_BUZZER, buzOn ? HIGH : LOW);
    digitalWrite(RELAY_WARN, HIGH);

    if (!lastWarn) {
      SDLogger::logWarn(
        now,
        curA,
        tempDriverL,
        tempDriverR,
        driveSafety
      );
    }

  } else {
    digitalWrite(PIN_BUZZER, LOW);
    digitalWrite(RELAY_WARN, LOW);
  }

  lastWarn = (driveSafety == SafetyState::WARN);

  applyDriveOutput();

  if (faultLatched) {
    driveSafe();
    digitalWrite(PIN_DRV_ENABLE, LOW);
    SDLogger::flush();
    return;
  }

  systemState = SystemState::ACTIVE;
  digitalWrite(PIN_DRV_ENABLE, HIGH);

  SDLogger::log(now);
  SDLogger::flush();

  wdDriveOK = true;
  wdBladeOK = true;
}
