// ========================================================================================
// SDLogger.cpp  (CONTEXT-BASED / HARDENED)
// ========================================================================================

#include "SDLogger.h"
#include "PinMap.h"
#include "SystemConfig.h"
#include "RuntimeContext.h"

// ========================================================================================
// GLOBAL CONTEXT
// ========================================================================================
extern RuntimeContext g_ctx;

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

static bool openNewFile() {

  char fname[13];
  for (uint16_t i = 0; i < 1000; i++) {
    sprintf(fname, "LOG%03u.CSV", i);
    if (!SD.exists(fname)) {
      logFile = SD.open(fname, FILE_WRITE);
      return (bool)logFile;
    }
  }
  return false;
}

// ========================================================================================
// BEGIN / RECOVER
// ========================================================================================
bool SDLogger::begin() {

  sdReady = false;
  wrIdx = rdIdx = logCount = 0;
  lastLog_ms = 0;

  if (!SD.begin(PIN_SD_CS)) {
    return false;
  }

  if (!openNewFile()) {
    return false;
  }

  writeHeader();
  sdReady = true;
  return true;
}

static void tryRecover() {

  if (sdReady && logFile) return;

  logFile.close();
  sdReady = false;

  if (!SD.begin(PIN_SD_CS)) {
    return;
  }

  if (!openNewFile()) {
    return;
  }

  writeHeader();
  sdReady = true;
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

  if (!sdReady) {
    tryRecover();
    return;
  }

  if (now - lastLog_ms < SD_LOG_PERIOD_MS) return;
  lastLog_ms = now;

  if (logCount >= SD_LOG_BUFFER_SIZE) return;

  SDLogRecord &r = logBuf[wrIdx];

  r.time_ms = now;

  // currents
  for (uint8_t i = 0; i < 4; i++) {
    r.curA[i] = g_ctx.curA[i];
  }

  r.curMax = max(
    max(g_ctx.curA[0], g_ctx.curA[1]),
    max(g_ctx.curA[2], g_ctx.curA[3])
  );

  // temps / voltage
  r.tempDriverL = g_ctx.tempDriverL;
  r.tempDriverR = g_ctx.tempDriverR;
  r.engineVolt  = g_ctx.engineVolt;

  // pwm
  r.pwmL = g_ctx.curL;
  r.pwmR = g_ctx.curR;

  // states
  r.systemState = (uint8_t)g_ctx.systemState;
  r.driveState  = (uint8_t)g_ctx.driveState;
  r.bladeState  = (uint8_t)g_ctx.bladeState;
  r.safetyState = (uint8_t)g_ctx.driveSafety;
  r.driveEvent  = (uint8_t)g_ctx.lastDriveEvent;

  r.faultLatched = g_ctx.faultLatched ? 1 : 0;

  wrIdx = (wrIdx + 1) % SD_LOG_BUFFER_SIZE;
  logCount++;
}

// ========================================================================================
// FLUSH (SLOW PATH — TIME BUDGETED)
// ========================================================================================
void SDLogger::flush() {

  if (!sdReady || !logFile || logCount == 0) {
    tryRecover();
    return;
  }

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

  (void)curA;   // kept for API compatibility
  (void)tempL;
  (void)tempR;
  (void)safety;

  if (!sdReady || !logFile) {
    tryRecover();
    return;
  }

  float curMax = max(
    max(g_ctx.curA[0], g_ctx.curA[1]),
    max(g_ctx.curA[2], g_ctx.curA[3])
  );

  // format: time,WARN,curMax,tempL,tempR,safety
  logFile.print(now);
  logFile.print(F(",WARN,"));
  logFile.print(curMax, 2);
  logFile.print(',');
  logFile.print(g_ctx.tempDriverL);
  logFile.print(',');
  logFile.print(g_ctx.tempDriverR);
  logFile.print(',');
  logFile.println((uint8_t)g_ctx.driveSafety);

  logFile.flush();   // WARN = event สำคัญ
}
