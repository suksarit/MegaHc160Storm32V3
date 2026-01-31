// ========================================================================================
// SDLogger.cpp
// ========================================================================================

#include "SDLogger.h"

// ========================================================================================
// EXTERN RUNTIME DATA (มาจาก MegaHc160Storm32.ino)
// ========================================================================================
extern float     curA[4];
extern float     engineVolt;
extern int16_t   curL;
extern int16_t   curR;
extern int16_t   tempDriverL;
extern int16_t   tempDriverR;

extern SystemState systemState;
extern DriveState  driveState;
extern BladeState  bladeState;
extern SafetyState driveSafety;
extern DriveEvent  lastDriveEvent;

extern bool faultLatched;

// ========================================================================================
// INTERNAL STATE
// ========================================================================================
static File logFile;
static bool sdReady = false;
static uint32_t lastLog_ms = 0;

// ring buffer
static SDLogRecord logBuf[SD_LOG_BUFFER_SIZE];
static uint8_t wrIdx = 0;
static uint8_t rdIdx = 0;
static uint8_t logCount = 0;

// ========================================================================================
// INTERNAL HELPERS
// ========================================================================================
static void writeHeader() {

  if (!logFile) return;

  logFile.println(
    F("time_ms,"
      "curL1,curL2,curR1,curR2,"
      "curMax,"
      "tempDL,tempDR,"
      "volt24,"
      "pwmL,pwmR,"
      "system,drive,blade,safety,event,fault")
  );
  logFile.flush();
}

// ========================================================================================
// BEGIN
// ========================================================================================
bool SDLogger::begin() {

  sdReady = false;
  wrIdx = rdIdx = logCount = 0;

  if (!SD.begin(SD_LOG_CS_PIN)) {
    return false;
  }

  char fname[13];
  for (uint16_t i = 0; i < 1000; i++) {
    sprintf(fname, "LOG%03u.CSV", i);
    if (!SD.exists(fname)) {
      logFile = SD.open(fname, FILE_WRITE);
      break;
    }
  }

  if (!logFile) {
    return false;
  }

  writeHeader();
  sdReady = true;
  return true;
}

// ========================================================================================
// STATUS
// ========================================================================================
bool SDLogger::isReady() {
  return sdReady;
}

bool SDLogger::bufferFull() {
  return (logCount >= SD_LOG_BUFFER_SIZE);
}

uint8_t SDLogger::pending() {
  return logCount;
}

// ========================================================================================
// LOG (FAST PATH — TELEMETRY BUFFER ONLY)
// ========================================================================================
void SDLogger::log(uint32_t now) {

  if (!sdReady) return;
  if (now - lastLog_ms < SD_LOG_PERIOD_MS) return;
  lastLog_ms = now;

  if (logCount >= SD_LOG_BUFFER_SIZE) return;

  SDLogRecord &r = logBuf[wrIdx];

  r.time_ms = now;

  r.curA[0] = curA[0];
  r.curA[1] = curA[1];
  r.curA[2] = curA[2];
  r.curA[3] = curA[3];

  r.curMax = max(
    max(curA[0], curA[1]),
    max(curA[2], curA[3])
  );

  r.tempDriverL = tempDriverL;
  r.tempDriverR = tempDriverR;
  r.engineVolt  = engineVolt;

  r.pwmL = curL;
  r.pwmR = curR;

  r.systemState = (uint8_t)systemState;
  r.driveState  = (uint8_t)driveState;
  r.bladeState  = (uint8_t)bladeState;
  r.safetyState = (uint8_t)driveSafety;
  r.driveEvent  = (uint8_t)lastDriveEvent;

  r.faultLatched = faultLatched ? 1 : 0;

  wrIdx = (wrIdx + 1) % SD_LOG_BUFFER_SIZE;
  logCount++;
}

// ========================================================================================
// FLUSH (SLOW PATH — TIME BUDGETED)
// ========================================================================================
void SDLogger::flush() {

  if (!sdReady) return;
  if (!logFile) return;
  if (logCount == 0) return;

  uint32_t start_us = micros();

  SDLogRecord &r = logBuf[rdIdx];

  logFile.print(r.time_ms); logFile.print(',');

  logFile.print(r.curA[0], 2); logFile.print(',');
  logFile.print(r.curA[1], 2); logFile.print(',');
  logFile.print(r.curA[2], 2); logFile.print(',');
  logFile.print(r.curA[3], 2); logFile.print(',');

  logFile.print(r.curMax, 2); logFile.print(',');

  logFile.print(r.tempDriverL); logFile.print(',');
  logFile.print(r.tempDriverR); logFile.print(',');

  logFile.print(r.engineVolt, 2); logFile.print(',');

  logFile.print(r.pwmL); logFile.print(',');
  logFile.print(r.pwmR); logFile.print(',');

  logFile.print(r.systemState); logFile.print(',');
  logFile.print(r.driveState);  logFile.print(',');
  logFile.print(r.bladeState);  logFile.print(',');
  logFile.print(r.safetyState); logFile.print(',');
  logFile.print(r.driveEvent);  logFile.print(',');
  logFile.println(r.faultLatched);

  rdIdx = (rdIdx + 1) % SD_LOG_BUFFER_SIZE;
  logCount--;

  static uint8_t flushCnt = 0;
  if (++flushCnt >= 20) {
    flushCnt = 0;
    logFile.flush();
  }

  if (micros() - start_us > SD_LOG_WRITE_BUDGET_US) {
    return;
  }
}

// ========================================================================================
// WARN EVENT LOG (IMMEDIATE, EVENT-BASED)
// ========================================================================================
void SDLogger::logWarn(uint32_t now,
                       const float curA[4],
                       int16_t tempL,
                       int16_t tempR,
                       SafetyState safety) {

  if (!sdReady || !logFile) return;

  float curMax = max(
    max(curA[0], curA[1]),
    max(curA[2], curA[3])
  );

  // format: time,WARN,curMax,tempL,tempR,safety
  logFile.print(now);
  logFile.print(F(",WARN,"));
  logFile.print(curMax, 2);
  logFile.print(',');
  logFile.print(tempL);
  logFile.print(',');
  logFile.print(tempR);
  logFile.print(',');
  logFile.println((uint8_t)safety);

  logFile.flush();   // WARN = event สำคัญ
}
