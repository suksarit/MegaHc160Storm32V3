// ========================================================================================
// SystemConfig.h
// Central system-wide configuration constants
// SINGLE SOURCE OF TRUTH
// ========================================================================================

#pragma once
#include <Arduino.h>

// ============================================================================
// PWM CONFIGURATION
// ============================================================================
constexpr uint16_t PWM_TOP = 1067;  // ~15 kHz @ 16 MHz (Timer3/4/5)
constexpr uint16_t PWM_MIN = 0;

// ============================================================================
// LOOP / TASK TIME BUDGET (ms)
// ============================================================================
constexpr uint16_t BUDGET_SENSORS_MS = 5;
constexpr uint16_t BUDGET_COMMS_MS   = 3;
constexpr uint16_t BUDGET_DRIVE_MS   = 2;
constexpr uint16_t BUDGET_BLADE_MS   = 2;
constexpr uint16_t BUDGET_LOOP_MS    = 20;

// ============================================================================
// IBUS CHANNEL MAP (GLOBAL)
// ============================================================================
constexpr uint8_t CH_STEER    = 1;
constexpr uint8_t CH_THROTTLE = 2;
constexpr uint8_t CH_ENGINE   = 8;
constexpr uint8_t CH_IGNITION = 6;
constexpr uint8_t CH_RESET    = 7;
constexpr uint8_t CH_STARTER  = 10;

// ============================================================================
// COMMUNICATION / WATCHDOG
// ============================================================================
constexpr uint32_t IBUS_TIMEOUT_MS   = 300;    // hard lost
constexpr uint32_t LOGIC_WDT_MS      = 500;    // logic watchdog
constexpr uint32_t SENSOR_WARMUP_MS  = 2000;   // boot sensor warmup

// ============================================================================
// DRIVE / SAFETY TIMING
// ============================================================================
constexpr uint32_t LIMP_RECOVER_MS             = 1000;
constexpr uint32_t DRIVE_SOFT_STOP_TIMEOUT_MS  = 1500;
constexpr uint32_t BLADE_SOFT_STOP_TIMEOUT_MS  = 3000;
constexpr uint16_t REVERSE_DEADTIME_MS         = 150;
constexpr uint16_t PWM_DEADTIME_US             = 8;   // 5–10 µs แนะนำ

// ============================================================================
// AUTO REVERSE
// ============================================================================
constexpr uint8_t  MAX_AUTO_REVERSE       = 2;
constexpr uint16_t AUTO_REV_PWM            = 300;
constexpr uint32_t AUTO_REV_MS             = 350;
constexpr uint32_t AUTO_REV_COOLDOWN_MS    = 1500;

// ============================================================================
// CURRENT THRESHOLDS
// ============================================================================
constexpr int16_t CUR_WARN_A  = 55;
constexpr int16_t CUR_LIMP_A  = 75;
constexpr float   CUR_SPIKE_A = 120.0f;

constexpr int16_t CUR_MIN_PLAUSIBLE = -10;
constexpr int16_t CUR_MAX_PLAUSIBLE = 150;
constexpr float   CUR_LPF_ALPHA     = 0.12f;

// ============================================================================
// CURRENT SENSOR CALIBRATION (GLOBAL, SHARED)
// ============================================================================
extern float g_acsOffsetV[4];   // L1, L2, R1, R2

// ============================================================================
// CURRENT-BASED TORQUE LIMIT
// ============================================================================
constexpr float TORQUE_LIMIT_A        = 70.0f;   // เริ่มจำกัด
constexpr float TORQUE_HARD_CUT_A     = 95.0f;   // emergency ก่อน HW trip
constexpr float TORQUE_RECOVER_A      = 60.0f;   // คืนกำลัง

constexpr float TORQUE_REDUCE_GAIN    = 0.015f;
constexpr float TORQUE_RECOVER_GAIN   = 0.008f;

// ============================================================================
// TEMPERATURE THRESHOLDS
// ============================================================================
constexpr int16_t TEMP_WARN_C          = 70;
constexpr int16_t TEMP_LIMP_C          = 85;
constexpr int16_t TEMP_TRIP_C          = 95;

constexpr int16_t TORQUE_TEMP_START_C  = 65;
constexpr int16_t TORQUE_TEMP_MAX_C    = 90;
constexpr float   TORQUE_TEMP_MIN_SCALE = 0.30f;

// ============================================================================
// ENGINE / VOLTAGE
// ============================================================================
constexpr float    ENGINE_RUNNING_VOLT      = 27.2f;
constexpr float    ENGINE_STOP_VOLT         = 25.5f;
constexpr uint32_t ENGINE_CONFIRM_MS        = 900;

constexpr uint32_t ENGINE_RESTART_GUARD_MS  = 3000;
constexpr uint32_t STARTER_MAX_MS           = 3000;

// ============================================================================
// VOLTAGE SENSE CONFIG
// ============================================================================

// ADS1115
constexpr float ADS_LSB_VOLT = 4.096f / 32768.0f;

// Voltage divider
constexpr float VOLT_DIV_RATIO = (150.0f + 33.0f) / 33.0f;

constexpr uint8_t VOLTAGE_ADS_CHANNEL = 0;

constexpr float VOLT_MAX_PLAUSIBLE = 40.0f;
constexpr float VOLT_LPF_ALPHA    = 0.15f;

// ============================================================================
// VOLTAGE WARNING LEVEL
// ============================================================================
constexpr float V_WARN_LOW      = 24.0f;
constexpr float V_WARN_CRITICAL = 23.0f;

// ============================================================================
// RTD (PT100)
// ============================================================================
constexpr float RTD_RNOMINAL = 100.0f;
constexpr float RTD_RREF     = 430.0f;

// ============================================================================
// FAN CONTROL
// ============================================================================
constexpr uint8_t FAN_MIN_PWM  = 80;
constexpr uint8_t FAN_IDLE_PWM = 50;
constexpr uint8_t FAN_PWM_HYST = 8;

constexpr int16_t FAN_L_START_C = 55;
constexpr int16_t FAN_L_FULL_C  = 85;

constexpr int16_t FAN_R_START_C = 60;
constexpr int16_t FAN_R_FULL_C  = 88;
