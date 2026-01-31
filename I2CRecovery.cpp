// ========================================================================================
// I2CRecovery.cpp
// ========================================================================================
#include "I2CRecovery.h"
#include <Wire.h>

bool I2CRecovery::recover() {

  constexpr uint8_t SDA_PIN = 20;
  constexpr uint8_t SCL_PIN = 21;

  TWCR &= ~(_BV(TWEN));

  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  delayMicroseconds(10);

  if (digitalRead(SDA_PIN) == HIGH) {
    TWCR |= _BV(TWEN);
    Wire.begin();
    return true;
  }

  pinMode(SCL_PIN, OUTPUT);
  for (uint8_t i = 0; i < 9; i++) {
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(10);
    if (digitalRead(SDA_PIN) == HIGH) break;
  }

  pinMode(SDA_PIN, OUTPUT);
  digitalWrite(SDA_PIN, LOW);
  delayMicroseconds(10);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SDA_PIN, HIGH);

  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);

  TWCR |= _BV(TWEN);
  Wire.begin();

  return digitalRead(SDA_PIN) == HIGH;
}
