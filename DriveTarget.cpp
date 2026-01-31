// ========================================================================================
// DriveTarget.cpp
// ========================================================================================
#include "DriveTarget.h"
#include "RuntimeContext.h"
#include "SystemConfig.h"
#include <IBusBM.h>

// ================= EXTERN OBJECTS =================
// IBus ยังเป็น device global (ไม่ใช่ runtime state)
extern IBusBM ibus;

// runtime context (ONE owner อยู่ที่ RuntimeContext.cpp)
extern RuntimeContext g_ctx;

// ================= CONSTANT =================
extern const uint8_t CH_THROTTLE;
extern const uint8_t CH_STEER;

// ================= IMPLEMENT =================
void DriveTarget::update() {

  // ---------------- SAFETY GATE ----------------
  if (g_ctx.faultLatched ||
      g_ctx.driveSafety == SafetyState::EMERGENCY ||
      g_ctx.systemState != SystemState::ACTIVE) {

    g_ctx.targetL = 0;
    g_ctx.targetR = 0;
    return;
  }

  // ---------------- AXIS MAPPING ----------------
  auto mapAxis = [](int16_t v) -> int16_t {

    constexpr int16_t IN_MIN  = 1000;
    constexpr int16_t IN_MAX  = 2000;
    constexpr int16_t DB_MIN  = 1450;
    constexpr int16_t DB_MAX  = 1550;
    constexpr int16_t OUT_MAX = (int16_t)PWM_TOP;

    // deadzone
    if (v >= DB_MIN && v <= DB_MAX) return 0;

    if (v < DB_MIN) {
      long m = map(v, IN_MIN, DB_MIN, -OUT_MAX, 0);
      return constrain(m, -OUT_MAX, 0);
    }

    long m = map(v, DB_MAX, IN_MAX, 0, OUT_MAX);
    return constrain(m, 0, OUT_MAX);
  };

  // ---------------- READ IBUS ----------------
  int16_t thr = mapAxis(ibus.readChannel(CH_THROTTLE));
  int16_t str = mapAxis(ibus.readChannel(CH_STEER));

  // ---------------- BLEND LOGIC (UNCHANGED) ----------------
  float absThr = abs(thr);
  float blend;

  if (absThr <= 150)        blend = 0.0f;
  else if (absThr >= 450)   blend = 1.0f;
  else                      blend = (absThr - 150.0f) / 300.0f;

  float arcL = thr + str;
  float arcR = thr - str;

  float k = abs(str) / (float)PWM_TOP;
  if (str < 0) k = -k;

  float diffL = thr * (1.0f + k);
  float diffR = thr * (1.0f - k);

  float outL = arcL  * (1.0f - blend) + diffL * blend;
  float outR = arcR  * (1.0f - blend) + diffR * blend;

  // ---------------- NORMALIZE ----------------
  float maxMag = max(abs(outL), abs(outR));
  if (maxMag > PWM_TOP) {
    float scale = (float)PWM_TOP / maxMag;
    outL *= scale;
    outR *= scale;
  }

  // ---------------- OUTPUT ----------------
  g_ctx.targetL = (int16_t)outL;
  g_ctx.targetR = (int16_t)outR;
}
