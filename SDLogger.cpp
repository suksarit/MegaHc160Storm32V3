// ========================================================================================
// SDLogger.cpp  (PHASE 1 / CONTEXT-BASED / HARDENED)
// ========================================================================================
#include "SDLogger.h"
#include "SystemConfig.h"
#include "RuntimeContext.h"
#include "FaultManager.h"

// ========================================================================================
extern RuntimeContext g_ctx;

// ========================================================================================
// INTERNAL STATE
// ========================================================================================
static File logFile;
static bool sdReady = false;
static uint32_t lastLog_ms = 0;
static uint32_t lastLoop_ms = 0;

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
    F("time,loop_dt,"
      "curL1,curL2,curR1,curR2,curMax,"
      "tempDL,tempDR,volt,"
      "pwmL,pwmR,"
      "system,drive,blade,safety,event,"
      "faultLatched,faultCode")
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
  lastLoop_ms = millis();

  if (!SD.begin(PIN_SD_CS)) return false;
  if (!openNewFile()) return false;

  writeHeader();
  sdReady = true;
  return true;
}

static void tryRecover() {

  if (sdReady && logFile) return;

  logFile.close();
  sdReady = false;

  if (!SD.begin(PIN_SD_CS)) return;
  if (!openNewFile()) return;

  writeHeader();
  sdReady = true;
}

// ========================================================================================
// STATUS
// ========================================================================================
bool SDLogger::isReady() { return sdReady; }
bool SDLogger::bufferFull() { return logCount >= SD_LOG_BUFFER_SIZE; }
uint8_t SDLogger::pending() { return logCount; }

// ========================================================================================
// LOG (FAST PATH)
// ========================================================================================
void SDLogger::log(uint32_t now) {

  if (!sdReady) { tryRecover(); return; }
  if (now - lastLog_ms < SD_LOG_PERIOD_MS) return;
  if (logCount >= SD_LOG_BUFFER_SIZE) return;

  SDLogRecord &r = logBuf[wrIdx];

  r.time_ms   = now;
  r.loop_dt_ms = (uint16_t)(now - lastLoop_ms);
  lastLoop_ms = now;

  for (uint8_t i = 0; i < 4; i++) r.curA[i] = g_ctx.curA[i];

  r.curMax = max(max(r.curA[0], r.curA[1]),
                 max(r.curA[2], r.curA[3]));

  r.tempDriverL = g_ctx.tempDriverL;
  r.tempDriverR = g_ctx.tempDriverR;
  r.engineVolt  = g_ctx.engineVolt;

  r.pwmL = g_ctx.curL;
  r.pwmR = g_ctx.curR;

  r.systemState = (uint8_t)g_ctx.systemState;
  r.driveState  = (uint8_t)g_ctx.driveState;
  r.bladeState  = (uint8_t)g_ctx.bladeState;
  r.safetyState = (uint8_t)g_ctx.driveSafety;
  r.driveEvent  = (uint8_t)g_ctx.lastDriveEvent;

  r.faultLatched = g_ctx.faultLatched ? 1 : 0;
  r.faultCode    = g_ctx.faultLatched ? (uint8_t)getActiveFault() : 0;

  wrIdx = (wrIdx + 1) % SD_LOG_BUFFER_SIZE;
  logCount++;
  lastLog_ms = now;
}

// ========================================================================================
// FLUSH (SLOW PATH)
// ========================================================================================
void SDLogger::flush() {

  if (!sdReady || !logFile || logCount == 0) {
    tryRecover();
    return;
  }

  SDLogRecord &r = logBuf[rdIdx];

  logFile.print(r.time_ms); logFile.print(',');
  logFile.print(r.loop_dt_ms); logFile.print(',');

  for (uint8_t i = 0; i < 4; i++) {
    logFile.print(r.curA[i], 2); logFile.print(',');
  }

  logFile.print(r.curMax, 2); logFile.print(',');
  logFile.print(r.tempDriverL); logFile.print(',');
  logFile.print(r.tempDriverR); logFile.print(',');
  logFile.print(r.engineVolt, 2); logFile.print(',');
  logFile.print(r.pwmL); logFile.print(',');
  logFile.print(r.pwmR); logFile.print(',');
  logFile.print(r.systemState); logFile.print(',');
  logFile.print(r.driveState); logFile.print(',');
  logFile.print(r.bladeState); logFile.print(',');
  logFile.print(r.safetyState); logFile.print(',');
  logFile.print(r.driveEvent); logFile.print(',');
  logFile.print(r.faultLatched); logFile.print(',');
  logFile.println(r.faultCode);

  rdIdx = (rdIdx + 1) % SD_LOG_BUFFER_SIZE;
  logCount--;
}
