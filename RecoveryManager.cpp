#include "RecoveryManager.h"
#include "SDLogger.h"

// ============================================================================
// CONFIG
// ============================================================================
#define AUTO_RECOVER_DELAY_MS   2000UL
#define RETRY_WINDOW_MS         30000UL
#define RETRY_MAX_COUNT         3

// ============================================================================
// PUBLIC
// ============================================================================
void RecoveryManager::begin() {

    mode = RecoveryMode::NONE;
    activeFault = FaultCode::NONE;
    faultTime = 0;
    manualConfirmed = false;

    retryFault = FaultCode::NONE;
    retryCount = 0;
    retryStartTime = 0;
}

void RecoveryManager::onFault(FaultCode fault) {

    uint32_t now = millis();

    activeFault = fault;
    faultTime = now;
    manualConfirmed = false;

    // ---- BLACKLIST (HARD NO RECOVERY) ----
    if (isBlacklisted(fault)) {
        mode = RecoveryMode::LOCKOUT;
        logEvent("BLACKLIST_LOCKOUT");
        return;
    }

    // ---- RETRY LIMIT ----
    if (!checkRetry(fault, now)) {
        mode = RecoveryMode::LOCKOUT;
        logEvent("RETRY_LOCKOUT");
        return;
    }

    decideMode();
    logEvent("ENTER_RECOVERY");
}

void RecoveryManager::update(uint32_t now) {

    if (mode == RecoveryMode::AUTO) {
        if (now - faultTime >= AUTO_RECOVER_DELAY_MS) {
            logEvent("AUTO_RECOVER_OK");
            activeFault = FaultCode::NONE;
            mode = RecoveryMode::NONE;
        }
    }
}

bool RecoveryManager::isRecoveryAllowed() const {

    return (mode == RecoveryMode::NONE) ||
           (mode == RecoveryMode::AUTO) ||
           (mode == RecoveryMode::MANUAL && manualConfirmed);
}

bool RecoveryManager::needsManualConfirm() const {
    return (mode == RecoveryMode::MANUAL && !manualConfirmed);
}

bool RecoveryManager::isLockedOut() const {
    return (mode == RecoveryMode::LOCKOUT);
}

void RecoveryManager::confirmManualRecovery() {

    if (mode == RecoveryMode::MANUAL) {
        manualConfirmed = true;
        logEvent("MANUAL_CONFIRM");
        activeFault = FaultCode::NONE;
        mode = RecoveryMode::NONE;
    }
}

RecoveryMode RecoveryManager::getMode() const {
    return mode;
}

FaultCode RecoveryManager::getFault() const {
    return activeFault;
}

// ============================================================================
// PRIVATE
// ============================================================================
void RecoveryManager::decideMode() {

    switch (activeFault) {

        // ---- AUTO RECOVERY ----
        case FaultCode::OVER_CURRENT:
            mode = RecoveryMode::AUTO;
            break;

        // ---- MANUAL CONFIRM REQUIRED ----
        case FaultCode::IBUS_LOST:
        case FaultCode::COMMS_TIMEOUT:
        case FaultCode::DRIVE_TIMEOUT:
        case FaultCode::BLADE_TIMEOUT:
            mode = RecoveryMode::MANUAL;
            break;

        // ---- HARD LOCKOUT ----
        default:
            mode = RecoveryMode::LOCKOUT;
            break;
    }
}

bool RecoveryManager::isBlacklisted(FaultCode f) {

    switch (f) {
        case FaultCode::OVER_TEMP:
        case FaultCode::TEMP_SENSOR_FAULT:
        case FaultCode::CUR_SENSOR_FAULT:
        case FaultCode::VOLT_SENSOR_FAULT:
        case FaultCode::LOGIC_WATCHDOG:
        case FaultCode::LOOP_OVERRUN:
            return true;

        default:
            return false;
    }
}

bool RecoveryManager::checkRetry(FaultCode f, uint32_t now) {

    if (retryFault != f || (now - retryStartTime) > RETRY_WINDOW_MS) {
        retryFault = f;
        retryCount = 1;
        retryStartTime = now;
    } else {
        retryCount++;
    }

    SDLogger::logRecoveryRetry(f, retryCount);

    return (retryCount <= RETRY_MAX_COUNT);
}

void RecoveryManager::logEvent(const char* tag) {

    SDLogger::logRecoveryEvent(
        tag,
        activeFault,
        static_cast<uint8_t>(mode)
    );
}
