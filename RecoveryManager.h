#pragma once

#include <Arduino.h>
#include "FaultManager.h"
#include "SDLogger.h"

// ============================================================================
// RECOVERY MODE
// ============================================================================
enum class RecoveryMode : uint8_t {
  NONE,
  AUTO,
  MANUAL,
  LOCKOUT
};

// ============================================================================
// RECOVERY MANAGER
// ============================================================================
class RecoveryManager {
public:
  void begin();

  // เรียกเมื่อระบบเข้า SAFE (มี fault latch แล้ว)
  void onFault(FaultCode fault);

  // เรียกทุก loop
  void update(uint32_t now);

  bool isRecoveryAllowed() const;
  bool needsManualConfirm() const;
  bool isLockedOut() const;

  void confirmManualRecovery();

  RecoveryMode getMode() const;
  FaultCode    getFault() const;

private:
  // ---- state ----
  RecoveryMode mode = RecoveryMode::NONE;
  FaultCode    activeFault = FaultCode::NONE;
  uint32_t     faultTime = 0;
  bool         manualConfirmed = false;

  // ---- retry (Task 5.5) ----
  FaultCode retryFault = FaultCode::NONE;
  uint8_t   retryCount = 0;
  uint32_t  retryStartTime = 0;

  // ---- internal helpers ----
  void decideMode();
  bool isBlacklisted(FaultCode f);
  bool checkRetry(FaultCode f, uint32_t now);
  void logEvent(const char* tag);
};
