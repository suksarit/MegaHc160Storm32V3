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

#include "PinMap.h"
#include "SystemTypes.h"
#include "SystemConfig.h"
#include "SafetyManager.h"
#include "FaultManager.h"
#include "SDLogger.h"
#include "TempSensor.h"
#include "VoltageSensor.h"
#include "CurrentSensor.h"
#include "DriveStateMachine.h"
#include "MotorOutput.h"

// ========================================================================================
// HARDWARE (NON-DRIVE PERIPHERALS – KEEP FEATURE)
// ========================================================================================
constexpr uint8_t PIN_DRV_ENABLE   = 33;
constexpr uint8_t SERVO_ENGINE_PIN = 9;
constexpr uint8_t MAX_CS_L         = 49;
constexpr uint8_t MAX_CS_R         = 48;
constexpr uint8_t PIN_BUZZER       = 30;
constexpr uint8_t RELAY_WARN       = 31;
constexpr uint8_t PIN_CUR_TRIP     = 32;

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
float g_acsOffsetV[4] = { 2.50f, 2.50f, 2.50f, 2.50f };

// ========================================================================================
// DRIVE VALUES
// ========================================================================================
int16_t targetL = 0;
int16_t targetR = 0;
int16_t curL    = 0;
int16_t curR    = 0;

// ========================================================================================
// ENGINE
// ========================================================================================
Servo bladeServo;
float engineVolt = 0.0f;

// ========================================================================================
// SENSOR RUNTIME
// ========================================================================================
float   curA[4]        = {0};
int16_t tempDriverL    = 0;
int16_t tempDriverR    = 0;

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
// FUNCTION PROTOTYPES
// ========================================================================================
void updateComms(uint32_t now);
void updateSensors(uint32_t now);

// ========================================================================================
// PWM INIT (KEEP FEATURE – TIMER3 / TIMER4 @15kHz)
// ========================================================================================
void initMotorPWM() {

  TCCR3A = 0; TCCR3B = 0;
  TCCR3A = (1 << COM3A1) | (1 << COM3B1) | (1 << WGM31);
  TCCR3B = (1 << WGM33)  | (1 << WGM32)  | (1 << CS30);
  ICR3 = PWM_TOP;
  OCR3A = 0; OCR3B = 0;

  TCCR4A = 0; TCCR4B = 0;
  TCCR4A = (1 << COM4A1) | (1 << COM4B1) | (1 << WGM41);
  TCCR4B = (1 << WGM43)  | (1 << WGM42)  | (1 << CS40);
  ICR4 = PWM_TOP;
  OCR4A = 0; OCR4B = 0;
}

// ========================================================================================
// HARD CUT (USED BY SAFETY & MOTOROUTPUT)
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

  float curMax = max(max(curA[0], curA[1]), max(curA[2], curA[3]));

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
// TORQUE LIMIT (CURRENT-BASED)
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

  if (curLeft > TORQUE_HARD_CUT_A || curRight > TORQUE_HARD_CUT_A) {
    lastDriveEvent = DriveEvent::WHEEL_LOCK;
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

  for (uint8_t i = 0; i < 4; i++) curA[i] = CurrentSensor::get(i);

  if (!TempSensor::update(tempDriverL, tempDriverR, now)) {
    latchFault(FaultCode::TEMP_SENSOR_FAULT);
    return;
  }

  if (!VoltageSensor::update(engineVolt, now)) {
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
  pinMode(PIN_CUR_TRIP, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(RELAY_WARN, OUTPUT);

  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(RELAY_WARN, LOW);
  digitalWrite(PIN_DRV_ENABLE, LOW);

  initMotorPWM();
  driveSafe();

  bladeServo.attach(SERVO_ENGINE_PIN);
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
      SDLogger::logWarn(now, curA, tempDriverL, tempDriverR, driveSafety);
    }

  } else {
    digitalWrite(PIN_BUZZER, LOW);
    digitalWrite(RELAY_WARN, LOW);
  }

  lastWarn = (driveSafety == SafetyState::WARN);

  MotorOutput::apply(now);

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
