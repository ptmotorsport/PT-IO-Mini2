#include "dbw_mode.h"

namespace {

struct DbwCalState {
  bool active;
  uint8_t mode;
  uint8_t stage;
  uint8_t tuneStep;
  uint8_t tuneStableSamples;
  uint8_t tunePassIndex;
  uint8_t tuneTrialIndex;
  uint8_t tuneSuccessCount;
  uint8_t savedControlMode;
  bool tuneControlModeForced;
  uint32_t stageStartMs;
  uint32_t tunePassStartMs;
  uint32_t tuneStepStartMs;
  uint32_t pedalClosedSum[2];
  uint32_t pedalClosedSamples;
  uint16_t pedalOpenExtreme[2];
  bool pedalOpenInit;
  uint32_t throttleClosedSum[2];
  uint32_t throttleClosedSamples;
  uint16_t throttleOpenExtreme[2];
  bool throttleOpenInit;
  uint16_t pedalClosedBaseline[2];
  uint16_t throttleClosedBaseline[2];
  uint32_t openResponseMs;
  uint32_t closeResponseMs;
  uint16_t tuneTargetPermil;
  uint16_t tuneMaxThrottlePermil;
  uint16_t tunePrevThrottlePermil;
  uint16_t tuneStepStartThrottlePermil;
  uint16_t tuneMaxOvershootPermil;
  uint16_t tuneMaxOvershootRelPermil;
  uint16_t tuneWorstSettleMs;
  uint16_t tuneLastSteadyErrorPermil;
  uint16_t tuneKp_x1000;
  uint16_t tuneKi_x1000;
  uint16_t tuneKd_x1000;
  uint16_t tuneOrigKp_x1000;
  uint16_t tuneOrigKi_x1000;
  uint16_t tuneOrigKd_x1000;
  uint16_t tuneKpSamples[3];
  uint16_t tuneKiSamples[3];
  uint16_t tuneKdSamples[3];
  float tuneIntegral;
  float tuneLastError;
};

static DbwCalState cal = {};

constexpr uint8_t DBW_CAL_MODE_FULL = 0;
constexpr uint8_t DBW_CAL_MODE_PEDAL = 1;
constexpr uint8_t DBW_CAL_MODE_THROTTLE = 2;
constexpr uint8_t DBW_CAL_MODE_AUTOTUNE = 3;
constexpr uint8_t DBW_OPT_INH_INVERT = 0x01;
constexpr uint8_t DBW_OPT_CONTROL_SHIFT = 1;
constexpr uint8_t DBW_OPT_CONTROL_MASK = 0x06;
constexpr uint8_t DBW_OPT_DIN_SHIFT = 3;
constexpr uint8_t DBW_OPT_DIN_MASK = 0x38;
constexpr uint8_t DBW_AUTOTUNE_RESULT_NONE = 0;
constexpr uint8_t DBW_AUTOTUNE_RESULT_RUNNING = 1;
constexpr uint8_t DBW_AUTOTUNE_RESULT_PASS = 2;
constexpr uint8_t DBW_AUTOTUNE_RESULT_FAIL = 3;

static uint16_t clampDuty(uint32_t value) {
  if (value > 255UL) {
    return 255U;
  }
  return static_cast<uint16_t>(value);
}

static uint16_t normalizeRaw(uint16_t raw, uint16_t closedRaw, uint16_t openRaw) {
  if (closedRaw == openRaw) {
    return 0U;
  }

  int32_t span = static_cast<int32_t>(openRaw) - static_cast<int32_t>(closedRaw);
  int32_t value = static_cast<int32_t>(raw) - static_cast<int32_t>(closedRaw);
  int32_t permil = (value * 1000L) / span;
  if (permil < 0L) {
    permil = 0L;
  } else if (permil > 1000L) {
    permil = 1000L;
  }
  return static_cast<uint16_t>(permil);
}

static uint16_t averagePermil(uint16_t a, uint16_t b) {
  return static_cast<uint16_t>((static_cast<uint32_t>(a) + static_cast<uint32_t>(b) + 1U) / 2U);
}

static uint16_t absDiffU16(uint16_t a, uint16_t b) {
  return (a > b) ? static_cast<uint16_t>(a - b) : static_cast<uint16_t>(b - a);
}

static void setMotorOutputs(const DbwConfig &cfg, uint8_t outputDuty[8], bool openDirection, uint16_t duty) {
  for (int i = 0; i < 4; i++) {
    outputDuty[i] = 0U;
  }

  uint8_t inhOnDuty = ((cfg.reserved & DBW_OPT_INH_INVERT) != 0U) ? 0U : 255U;

  // Keep both half-bridges enabled and PWM only one IN line per direction.
  // OUT1 = HB1 IN, OUT2 = HB1 INH, OUT3 = HB2 IN, OUT4 = HB2 INH
  outputDuty[1] = inhOnDuty;
  outputDuty[3] = inhOnDuty;

  if (duty == 0U) {
    return;
  }

  duty = clampDuty(duty);
  if (openDirection) {
    outputDuty[0] = static_cast<uint8_t>(duty);
    outputDuty[2] = 0U;
  } else {
    outputDuty[0] = 0U;
    outputDuty[2] = static_cast<uint8_t>(duty);
  }
}

static void zeroMotorOutputs(uint8_t outputDuty[8]) {
  for (int i = 0; i < 4; i++) {
    outputDuty[i] = 0U;
  }
}

static bool endpointsValid(const DbwConfig &cfg) {
  for (int i = 0; i < 2; i++) {
    if (cfg.pedalClosedRaw[i] == cfg.pedalOpenRaw[i]) {
      return false;
    }
    if (cfg.throttleClosedRaw[i] == cfg.throttleOpenRaw[i]) {
      return false;
    }
  }
  return true;
}

static bool pedalEndpointsValid(const DbwConfig &cfg) {
  for (int i = 0; i < 2; i++) {
    if (cfg.pedalClosedRaw[i] == cfg.pedalOpenRaw[i]) {
      return false;
    }
  }
  return true;
}

static bool throttleEndpointsValid(const DbwConfig &cfg) {
  for (int i = 0; i < 2; i++) {
    if (cfg.throttleClosedRaw[i] == cfg.throttleOpenRaw[i]) {
      return false;
    }
  }
  return true;
}

static void refreshCalibratedFlag(DbwConfig &cfg) {
  cfg.calibrated = (pedalEndpointsValid(cfg) && throttleEndpointsValid(cfg)) ? 1U : 0U;
}

static void captureClosedBaseline(uint32_t sum[2], uint32_t &samples, const uint16_t raw[2]) {
  sum[0] += raw[0];
  sum[1] += raw[1];
  if (samples != 0xFFFFFFFFUL) {
    samples++;
  }
}

static void updateExtremeFromBaseline(uint16_t extreme[2], bool &init, const uint16_t raw[2], const uint16_t baseline[2]) {
  for (int i = 0; i < 2; i++) {
    if (!init) {
      extreme[i] = raw[i];
    } else {
      uint16_t current = absDiffU16(raw[i], baseline[i]);
      uint16_t stored = absDiffU16(extreme[i], baseline[i]);
      if (current > stored) {
        extreme[i] = raw[i];
      }
    }
  }
  init = true;
}

static uint16_t calculateResponseWindow(const uint16_t raw[2], const uint16_t baseline[2], uint32_t elapsedMs) {
  uint16_t delta0 = absDiffU16(raw[0], baseline[0]);
  uint16_t delta1 = absDiffU16(raw[1], baseline[1]);
  uint16_t delta = averagePermil(delta0, delta1);
  if (delta > 120U) {
    return static_cast<uint16_t>(elapsedMs);
  }
  return 0U;
}

static uint16_t clampGainU16(uint32_t value, uint16_t minValue, uint16_t maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return static_cast<uint16_t>(value);
}

static void adjustTuneGainsFromMetrics(DbwCalState &state) {
  uint16_t overshoot = state.tuneMaxOvershootPermil;
  uint16_t settleMs = state.tuneWorstSettleMs;

  if (overshoot > 300U) {
    state.tuneKp_x1000 = clampGainU16((static_cast<uint32_t>(state.tuneKp_x1000) * 55UL) / 100UL, 90U, 900U);
    state.tuneKd_x1000 = clampGainU16((static_cast<uint32_t>(state.tuneKd_x1000) * 180UL) / 100UL, 8U, 500U);
    state.tuneKi_x1000 = clampGainU16((static_cast<uint32_t>(state.tuneKi_x1000) * 60UL) / 100UL, 1U, 250U);
    return;
  }

  if (overshoot > 140U) {
    state.tuneKp_x1000 = clampGainU16((static_cast<uint32_t>(state.tuneKp_x1000) * 80UL) / 100UL, 120U, 900U);
    state.tuneKd_x1000 = clampGainU16((static_cast<uint32_t>(state.tuneKd_x1000) * 130UL) / 100UL, 5U, 450U);
    state.tuneKi_x1000 = clampGainU16((static_cast<uint32_t>(state.tuneKi_x1000) * 85UL) / 100UL, 1U, 300U);
    return;
  }

  if (settleMs > 1200U && overshoot < 60U) {
    state.tuneKp_x1000 = clampGainU16((static_cast<uint32_t>(state.tuneKp_x1000) * 115UL) / 100UL, 120U, 900U);
    state.tuneKi_x1000 = clampGainU16((static_cast<uint32_t>(state.tuneKi_x1000) * 120UL) / 100UL, 1U, 300U);
    return;
  }

  if (settleMs < 450U && overshoot < 25U) {
    state.tuneKi_x1000 = clampGainU16((static_cast<uint32_t>(state.tuneKi_x1000) * 110UL) / 100UL, 1U, 300U);
  }
}

static uint16_t aggregateMedianOrAverage(const uint16_t *values, uint8_t count) {
  if (count == 0U) {
    return 0U;
  }
  if (count == 1U) {
    return values[0];
  }
  if (count == 2U) {
    return static_cast<uint16_t>((static_cast<uint32_t>(values[0]) + static_cast<uint32_t>(values[1])) / 2U);
  }

  uint16_t a = values[0];
  uint16_t b = values[1];
  uint16_t c = values[2];
  if (a > b) {
    uint16_t t = a; a = b; b = t;
  }
  if (b > c) {
    uint16_t t = b; b = c; c = t;
  }
  if (a > b) {
    uint16_t t = a; a = b; b = t;
  }
  return b;
}

static void resetTuneTrialState(DbwCalState &state,
                                uint32_t nowMs,
                                uint16_t throttlePos,
                                uint16_t kp_x1000,
                                uint16_t ki_x1000,
                                uint16_t kd_x1000) {
  state.tunePassIndex = 0U;
  state.tuneStep = 0U;
  state.tuneStableSamples = 0U;
  state.tunePassStartMs = nowMs;
  state.tuneStepStartMs = nowMs;
  state.tuneStepStartThrottlePermil = throttlePos;
  state.tuneMaxThrottlePermil = throttlePos;
  state.tunePrevThrottlePermil = throttlePos;
  state.tuneMaxOvershootPermil = 0U;
  state.tuneMaxOvershootRelPermil = 0U;
  state.tuneWorstSettleMs = 0U;
  state.tuneLastSteadyErrorPermil = 0U;
  state.tuneKp_x1000 = clampGainU16(kp_x1000, 90U, 900U);
  state.tuneKi_x1000 = clampGainU16(ki_x1000, 1U, 300U);
  state.tuneKd_x1000 = clampGainU16(kd_x1000, 5U, 500U);
  state.tuneIntegral = 0.0f;
  state.tuneLastError = 0.0f;
}

static void finaliseCalibration(DbwConfig &cfg) {
  if (cal.pedalClosedSamples == 0U || cal.throttleClosedSamples == 0U) {
    return;
  }

  for (int i = 0; i < 2; i++) {
    cfg.pedalClosedRaw[i] = static_cast<uint16_t>(cal.pedalClosedSum[i] / cal.pedalClosedSamples);
    cfg.pedalOpenRaw[i] = cal.pedalOpenExtreme[i];
    cfg.throttleClosedRaw[i] = static_cast<uint16_t>(cal.throttleClosedSum[i] / cal.throttleClosedSamples);
    cfg.throttleOpenRaw[i] = cal.throttleOpenExtreme[i];
  }

  uint32_t responseMs = cal.openResponseMs;
  if (cal.closeResponseMs > responseMs) {
    responseMs = cal.closeResponseMs;
  }
  if (responseMs == 0U) {
    responseMs = 250U;
  }

  uint32_t kp = 250000UL / responseMs;
  if (kp < 160UL) {
    kp = 160UL;
  }
  if (kp > 650UL) {
    kp = 650UL;
  }

  cfg.kp_x1000 = static_cast<uint16_t>(kp);
  cfg.ki_x1000 = static_cast<uint16_t>((kp / 12UL) < 5UL ? 5UL : (kp / 12UL));
  cfg.kd_x1000 = static_cast<uint16_t>((responseMs / 8UL) < 5UL ? 5UL : (responseMs / 8UL));
  refreshCalibratedFlag(cfg);
  if (cfg.calibrated != 0U) {
    cfg.enabled = 1U;
  }
}

static void resetCalibrationState(uint32_t nowMs, uint8_t mode) {
  cal.active = true;
  cal.mode = mode;
  cal.stage = 0U;
  cal.tuneStep = 0U;
  cal.tuneStableSamples = 0U;
  cal.tunePassIndex = 0U;
  cal.tuneTrialIndex = 0U;
  cal.tuneSuccessCount = 0U;
  cal.savedControlMode = DBW_CONTROL_APS;
  cal.tuneControlModeForced = false;
  cal.stageStartMs = nowMs;
  cal.tunePassStartMs = nowMs;
  cal.tuneStepStartMs = nowMs;
  cal.pedalClosedSum[0] = cal.pedalClosedSum[1] = 0UL;
  cal.pedalClosedSamples = 0U;
  cal.pedalOpenExtreme[0] = cal.pedalOpenExtreme[1] = 0U;
  cal.pedalOpenInit = false;
  cal.throttleClosedSum[0] = cal.throttleClosedSum[1] = 0UL;
  cal.throttleClosedSamples = 0U;
  cal.throttleOpenExtreme[0] = cal.throttleOpenExtreme[1] = 0U;
  cal.throttleOpenInit = false;
  cal.pedalClosedBaseline[0] = cal.pedalClosedBaseline[1] = 0U;
  cal.throttleClosedBaseline[0] = cal.throttleClosedBaseline[1] = 0U;
  cal.openResponseMs = 0U;
  cal.closeResponseMs = 0U;
  cal.tuneTargetPermil = 0U;
  cal.tuneMaxThrottlePermil = 0U;
  cal.tunePrevThrottlePermil = 0U;
  cal.tuneStepStartThrottlePermil = 0U;
  cal.tuneMaxOvershootPermil = 0U;
  cal.tuneMaxOvershootRelPermil = 0U;
  cal.tuneWorstSettleMs = 0U;
  cal.tuneLastSteadyErrorPermil = 0U;
  cal.tuneKp_x1000 = 0U;
  cal.tuneKi_x1000 = 0U;
  cal.tuneKd_x1000 = 0U;
  cal.tuneOrigKp_x1000 = 0U;
  cal.tuneOrigKi_x1000 = 0U;
  cal.tuneOrigKd_x1000 = 0U;
  cal.tuneKpSamples[0] = cal.tuneKpSamples[1] = cal.tuneKpSamples[2] = 0U;
  cal.tuneKiSamples[0] = cal.tuneKiSamples[1] = cal.tuneKiSamples[2] = 0U;
  cal.tuneKdSamples[0] = cal.tuneKdSamples[1] = cal.tuneKdSamples[2] = 0U;
  cal.tuneIntegral = 0.0f;
  cal.tuneLastError = 0.0f;

  if (mode == DBW_CAL_MODE_THROTTLE) {
    // Reuse existing pedal endpoints when only the throttle body is being calibrated.
    cal.pedalClosedSamples = 1U;
    cal.pedalClosedSum[0] = 0UL;
    cal.pedalClosedSum[1] = 0UL;
    cal.pedalOpenExtreme[0] = 0U;
    cal.pedalOpenExtreme[1] = 0U;
  }
}

}  // namespace

void dbwSetDefaults(DbwConfig &cfg) {
  for (int i = 0; i < 2; i++) {
    cfg.pedalClosedRaw[i] = 0U;
    cfg.pedalOpenRaw[i] = 0U;
    cfg.throttleClosedRaw[i] = 0U;
    cfg.throttleOpenRaw[i] = 0U;
  }
  cfg.kp_x1000 = 250U;
  cfg.ki_x1000 = 20U;
  cfg.kd_x1000 = 20U;
  cfg.pwmFreqHz = 2000U;
  cfg.deadbandPermil = 15U;
  cfg.sensorMismatchPermil = 80U;
  cfg.maxDuty = 180U;
  cfg.enabled = 0U;
  cfg.directionInvert = 0U;
  cfg.calibrated = 0U;
  cfg.reserved = 0U;
  dbwSetControlMode(cfg, DBW_CONTROL_APS);
  dbwSetDigitalInputChannel(cfg, 0U);
}

void dbwResetRuntime(DbwStatus &status) {
  status.state = DBW_STATE_IDLE;
  status.faultReason = DBW_FAULT_NONE;
  status.calStage = 0U;
  status.manualTargetActive = 0U;
  status.autoTuneResult = DBW_AUTOTUNE_RESULT_NONE;
  status.pedalPermil = 0U;
  status.throttlePermil = 0U;
  status.targetPermil = 0U;
  status.motorDuty = 0U;
  status.autoTunePass = 0U;
  status.autoTuneMaxOvershoot = 0U;
  status.autoTuneMaxSettleMs = 0U;
  status.autoTuneLastSteadyError = 0U;
  status.errorPermil = 0;
  status.calProgressPermil = 0U;
  status.lastUpdateMs = 0U;
  status.pidIntegral = 0.0f;
  status.pidLastError = 0.0f;
  status.pidStateValid = 0U;
}

void dbwBeginCalibration(DbwStatus &status, uint32_t nowMs) {
  resetCalibrationState(nowMs, DBW_CAL_MODE_FULL);
  status.state = DBW_STATE_CALIBRATING;
  status.faultReason = DBW_FAULT_NONE;
  status.calStage = 0U;
  status.manualTargetActive = 0U;
  status.autoTuneResult = DBW_AUTOTUNE_RESULT_NONE;
  status.targetPermil = 0U;
  status.motorDuty = 0U;
  status.errorPermil = 0;
  status.calProgressPermil = 0U;
  status.lastUpdateMs = nowMs;
  status.pidIntegral = 0.0f;
  status.pidLastError = 0.0f;
  status.pidStateValid = 0U;
}

void dbwBeginPedalCalibration(DbwStatus &status, uint32_t nowMs) {
  resetCalibrationState(nowMs, DBW_CAL_MODE_PEDAL);
  status.state = DBW_STATE_CALIBRATING;
  status.faultReason = DBW_FAULT_NONE;
  status.calStage = 0U;
  status.manualTargetActive = 0U;
  status.autoTuneResult = DBW_AUTOTUNE_RESULT_NONE;
  status.targetPermil = 0U;
  status.motorDuty = 0U;
  status.errorPermil = 0;
  status.calProgressPermil = 0U;
  status.lastUpdateMs = nowMs;
  status.pidIntegral = 0.0f;
  status.pidLastError = 0.0f;
  status.pidStateValid = 0U;
}

void dbwBeginThrottleCalibration(DbwStatus &status, uint32_t nowMs) {
  resetCalibrationState(nowMs, DBW_CAL_MODE_THROTTLE);
  status.state = DBW_STATE_CALIBRATING;
  status.faultReason = DBW_FAULT_NONE;
  status.calStage = 0U;
  status.manualTargetActive = 0U;
  status.autoTuneResult = DBW_AUTOTUNE_RESULT_NONE;
  status.targetPermil = 0U;
  status.motorDuty = 0U;
  status.errorPermil = 0;
  status.calProgressPermil = 0U;
  status.lastUpdateMs = nowMs;
  status.pidIntegral = 0.0f;
  status.pidLastError = 0.0f;
  status.pidStateValid = 0U;
}

void dbwBeginAutoTune(DbwStatus &status, uint32_t nowMs) {
  resetCalibrationState(nowMs, DBW_CAL_MODE_AUTOTUNE);
  status.state = DBW_STATE_CALIBRATING;
  status.faultReason = DBW_FAULT_NONE;
  status.calStage = 0U;
  status.manualTargetActive = 0U;
  status.autoTuneResult = DBW_AUTOTUNE_RESULT_RUNNING;
  status.targetPermil = 0U;
  status.motorDuty = 0U;
  status.autoTunePass = 0U;
  status.autoTuneMaxOvershoot = 0U;
  status.autoTuneMaxSettleMs = 0U;
  status.autoTuneLastSteadyError = 0U;
  status.errorPermil = 0;
  status.calProgressPermil = 0U;
  status.lastUpdateMs = nowMs;
  status.pidIntegral = 0.0f;
  status.pidLastError = 0.0f;
  status.pidStateValid = 0U;
}

void dbwAbortCalibration(DbwStatus &status, DbwConfig *cfg) {
  if (cfg != nullptr && cal.active && cal.mode == DBW_CAL_MODE_AUTOTUNE && cal.tuneControlModeForced) {
    dbwSetControlMode(*cfg, cal.savedControlMode);
    cal.tuneControlModeForced = false;
  }
  if (cal.active && cal.mode == DBW_CAL_MODE_AUTOTUNE && status.autoTuneResult == DBW_AUTOTUNE_RESULT_RUNNING) {
    status.autoTuneResult = DBW_AUTOTUNE_RESULT_FAIL;
  }
  cal.active = false;
  cal.mode = DBW_CAL_MODE_FULL;
  status.calStage = 0U;
  status.calProgressPermil = 0U;
  status.motorDuty = 0U;
  if (status.faultReason == DBW_FAULT_NONE) {
    status.state = DBW_STATE_IDLE;
  }
}

void dbwSetManualTarget(DbwStatus &status, bool enabled, uint16_t targetPermil) {
  status.manualTargetActive = enabled ? 1U : 0U;
  if (enabled) {
    status.targetPermil = (targetPermil > 1000U) ? 1000U : targetPermil;
  }
}

void dbwSetEnabled(DbwConfig &cfg, bool enabled) {
  cfg.enabled = enabled ? 1U : 0U;
}

void dbwSetMotorInvert(DbwConfig &cfg, bool invert) {
  cfg.directionInvert = invert ? 1U : 0U;
}

void dbwSetInhInverted(DbwConfig &cfg, bool invert) {
  if (invert) {
    cfg.reserved = static_cast<uint8_t>(cfg.reserved | DBW_OPT_INH_INVERT);
  } else {
    cfg.reserved = static_cast<uint8_t>(cfg.reserved & static_cast<uint8_t>(~DBW_OPT_INH_INVERT));
  }
}

bool dbwGetInhInverted(const DbwConfig &cfg) {
  return (cfg.reserved & DBW_OPT_INH_INVERT) != 0U;
}

void dbwSetControlMode(DbwConfig &cfg, uint8_t mode) {
  if (mode > DBW_CONTROL_CAN) {
    mode = DBW_CONTROL_APS;
  }
  cfg.reserved = static_cast<uint8_t>((cfg.reserved & static_cast<uint8_t>(~DBW_OPT_CONTROL_MASK)) |
                                       static_cast<uint8_t>((mode << DBW_OPT_CONTROL_SHIFT) & DBW_OPT_CONTROL_MASK));
}

uint8_t dbwGetControlMode(const DbwConfig &cfg) {
  uint8_t mode = static_cast<uint8_t>((cfg.reserved & DBW_OPT_CONTROL_MASK) >> DBW_OPT_CONTROL_SHIFT);
  if (mode > DBW_CONTROL_CAN) {
    return DBW_CONTROL_APS;
  }
  return mode;
}

const char *dbwControlModeName(uint8_t mode) {
  switch (mode) {
    case DBW_CONTROL_APS: return "APS";
    case DBW_CONTROL_DIGITAL_IN: return "DIGITAL_IN";
    case DBW_CONTROL_CAN: return "CAN";
    default: return "UNKNOWN";
  }
}

void dbwSetDigitalInputChannel(DbwConfig &cfg, uint8_t channel) {
  if (channel > 7U) {
    channel = 7U;
  }
  cfg.reserved = static_cast<uint8_t>((cfg.reserved & static_cast<uint8_t>(~DBW_OPT_DIN_MASK)) |
                                       static_cast<uint8_t>((channel << DBW_OPT_DIN_SHIFT) & DBW_OPT_DIN_MASK));
}

uint8_t dbwGetDigitalInputChannel(const DbwConfig &cfg) {
  uint8_t channel = static_cast<uint8_t>((cfg.reserved & DBW_OPT_DIN_MASK) >> DBW_OPT_DIN_SHIFT);
  return (channel > 7U) ? 7U : channel;
}

const char *dbwStateName(uint8_t state) {
  switch (state) {
    case DBW_STATE_IDLE: return "idle";
    case DBW_STATE_ARMED: return "armed";
    case DBW_STATE_CALIBRATING: return "calibrating";
    case DBW_STATE_FAULT: return "fault";
    default: return "unknown";
  }
}

const char *dbwFaultName(uint8_t fault) {
  switch (fault) {
    case DBW_FAULT_NONE: return "none";
    case DBW_FAULT_SENSOR: return "sensor";
    case DBW_FAULT_CALIBRATION: return "calibration";
    case DBW_FAULT_RANGE: return "range";
    case DBW_FAULT_CONFIG: return "config";
    default: return "unknown";
  }
}

DbwServiceResult dbwService(DbwConfig &cfg,
                            DbwStatus &status,
                            uint32_t nowMs,
                            const uint16_t analogRaw[8],
                            uint8_t outputDuty[8],
                            uint16_t outputFreq[8],
                            bool externalTargetActive,
                            bool externalTargetValid,
                            uint16_t externalTargetPermil,
                            bool externalFreqActive,
                            uint16_t externalFreqHz) {
  DbwServiceResult result = {false, false};
  uint32_t prevUpdateMs = status.lastUpdateMs;
  uint16_t prevMotorDuty = status.motorDuty;
  status.lastUpdateMs = nowMs;

  for (int i = 0; i < 8; i++) {
    outputDuty[i] = 0U;
  }

  uint16_t driveFreq = cfg.pwmFreqHz;
  if (externalFreqActive) {
    driveFreq = externalFreqHz;
  }
  outputFreq[0] = driveFreq;
  outputFreq[1] = driveFreq;
  outputFreq[2] = driveFreq;
  outputFreq[3] = driveFreq;

  uint16_t pedal0 = 0U;
  uint16_t pedal1 = 0U;
  uint16_t throttle0 = 0U;
  uint16_t throttle1 = 0U;

  if (pedalEndpointsValid(cfg) && throttleEndpointsValid(cfg)) {
    pedal0 = normalizeRaw(analogRaw[6], cfg.pedalClosedRaw[0], cfg.pedalOpenRaw[0]);
    pedal1 = normalizeRaw(analogRaw[7], cfg.pedalClosedRaw[1], cfg.pedalOpenRaw[1]);
    throttle0 = normalizeRaw(analogRaw[4], cfg.throttleClosedRaw[0], cfg.throttleOpenRaw[0]);
    throttle1 = normalizeRaw(analogRaw[5], cfg.throttleClosedRaw[1], cfg.throttleOpenRaw[1]);
  }

  if (pedalEndpointsValid(cfg) && throttleEndpointsValid(cfg)) {
    if (absDiffU16(pedal0, pedal1) > cfg.sensorMismatchPermil ||
        absDiffU16(throttle0, throttle1) > cfg.sensorMismatchPermil) {
      status.state = DBW_STATE_FAULT;
      status.faultReason = DBW_FAULT_SENSOR;
      status.motorDuty = 0U;
      status.errorPermil = 0;
      status.pidIntegral = 0.0f;
      status.pidLastError = 0.0f;
      status.pidStateValid = 0U;
      zeroMotorOutputs(outputDuty);
      return result;
    }
  }

  if (cal.active) {
    status.state = DBW_STATE_CALIBRATING;
    status.calStage = static_cast<uint8_t>((cal.mode << 4U) | (cal.stage & 0x0FU));
    status.faultReason = DBW_FAULT_NONE;
    status.targetPermil = 0U;
    status.motorDuty = 0U;
    status.errorPermil = 0;

    uint32_t elapsed = nowMs - cal.stageStartMs;
    status.calProgressPermil = (elapsed >= 4000UL) ? 1000U : static_cast<uint16_t>((elapsed * 1000UL) / 4000UL);

    uint16_t pedalRaw[2] = {analogRaw[6], analogRaw[7]};
    uint16_t throttleRaw[2] = {analogRaw[4], analogRaw[5]};

    if (cal.mode == DBW_CAL_MODE_AUTOTUNE) {
      if (!endpointsValid(cfg) || cfg.calibrated == 0U) {
        if (cal.tuneControlModeForced) {
          dbwSetControlMode(cfg, cal.savedControlMode);
          cal.tuneControlModeForced = false;
        }
        cal.active = false;
        status.state = DBW_STATE_FAULT;
        status.faultReason = DBW_FAULT_CONFIG;
        status.autoTuneResult = DBW_AUTOTUNE_RESULT_FAIL;
        zeroMotorOutputs(outputDuty);
        result.outputsChanged = true;
        return result;
      }

      if (!cal.tuneControlModeForced) {
        cal.savedControlMode = dbwGetControlMode(cfg);
        if (cal.savedControlMode != DBW_CONTROL_APS) {
          dbwSetControlMode(cfg, DBW_CONTROL_APS);
          result.configChanged = true;
        }
        cal.tuneControlModeForced = true;
      }

      uint16_t pedalValue = averagePermil(pedal0, pedal1);
      uint16_t throttlePos = averagePermil(throttle0, throttle1);
      status.pedalPermil = pedalValue;
      status.throttlePermil = throttlePos;

      constexpr uint16_t kTuneTargets[4] = {150U, 300U, 500U, 250U};
      constexpr uint32_t kTuneStage0Ms = 1200UL;
      constexpr uint32_t kTuneStepTimeoutMs = 2200UL;
      constexpr uint16_t kTuneSettleBandPermil = 35U;
      constexpr uint32_t kTuneMinSettleMs = 250UL;
      constexpr uint8_t kTuneTrialCount = 3U;
      constexpr uint16_t kTuneMaxRelOvershootPermil = 700U;

      if (cal.stage == 0U) {
        status.targetPermil = 0U;
        status.motorDuty = 0U;
        zeroMotorOutputs(outputDuty);
        status.calProgressPermil = static_cast<uint16_t>(((nowMs - cal.stageStartMs) * 120UL) / kTuneStage0Ms);
        if (status.calProgressPermil > 120U) {
          status.calProgressPermil = 120U;
        }

        if ((nowMs - cal.stageStartMs) >= kTuneStage0Ms) {
          cal.stage = 1U;
          cal.tuneOrigKp_x1000 = cfg.kp_x1000;
          cal.tuneOrigKi_x1000 = cfg.ki_x1000;
          cal.tuneOrigKd_x1000 = cfg.kd_x1000;
          cal.tuneTrialIndex = 0U;
          cal.tuneSuccessCount = 0U;
          resetTuneTrialState(cal,
                              nowMs,
                              throttlePos,
                              cal.tuneOrigKp_x1000,
                              cal.tuneOrigKi_x1000,
                              cal.tuneOrigKd_x1000);
          status.autoTunePass = 1U;
        }

        result.outputsChanged = true;
        return result;
      }

      status.autoTunePass = static_cast<uint16_t>((cal.tuneTrialIndex * 2U) + cal.tunePassIndex + 1U);

      uint16_t target = kTuneTargets[cal.tuneStep < 4U ? cal.tuneStep : 3U];
      status.targetPermil = target;

      float dt = 0.005f;
      if (prevUpdateMs != 0U && nowMs > prevUpdateMs) {
        dt = static_cast<float>(nowMs - prevUpdateMs) / 1000.0f;
        if (dt < 0.005f) {
          dt = 0.005f;
        } else if (dt > 0.02f) {
          dt = 0.02f;
        }
      }

      int16_t errorPermil = static_cast<int16_t>(static_cast<int32_t>(target) - static_cast<int32_t>(throttlePos));
      float kp = static_cast<float>(cal.tuneKp_x1000) / 1000.0f;
      float ki = static_cast<float>(cal.tuneKi_x1000) / 1000.0f;
      float kd = static_cast<float>(cal.tuneKd_x1000) / 1000.0f;

      cal.tuneIntegral += static_cast<float>(errorPermil) * dt;
      if (cal.tuneIntegral > 1500.0f) {
        cal.tuneIntegral = 1500.0f;
      } else if (cal.tuneIntegral < -1500.0f) {
        cal.tuneIntegral = -1500.0f;
      }

      float derivative = (static_cast<float>(errorPermil) - cal.tuneLastError) / dt;
      cal.tuneLastError = static_cast<float>(errorPermil);

      float command = (kp * static_cast<float>(errorPermil)) + (ki * cal.tuneIntegral) + (kd * derivative);
      bool openDirection = (command >= 0.0f);
      if (cfg.directionInvert != 0U) {
        openDirection = !openDirection;
      }

      uint16_t duty = static_cast<uint16_t>(fabsf(command));
      uint16_t tuneDutyLimit = cfg.maxDuty;
      if (tuneDutyLimit > 120U) {
        tuneDutyLimit = 120U;
      }
      if (duty > tuneDutyLimit) {
        duty = tuneDutyLimit;
      }
      if (duty > 0U && duty < 18U) {
        duty = 18U;
      }

      status.errorPermil = errorPermil;
      status.motorDuty = duty;
      setMotorOutputs(cfg, outputDuty, openDirection, duty);

      if (throttlePos > cal.tuneMaxThrottlePermil) {
        cal.tuneMaxThrottlePermil = throttlePos;
      }

      if (throttlePos > target) {
        uint16_t overshoot = static_cast<uint16_t>(throttlePos - target);
        if (overshoot > cal.tuneMaxOvershootPermil) {
          cal.tuneMaxOvershootPermil = overshoot;
        }

        uint16_t stepAmplitude = (target > cal.tuneStepStartThrottlePermil)
                                   ? static_cast<uint16_t>(target - cal.tuneStepStartThrottlePermil)
                                   : static_cast<uint16_t>(cal.tuneStepStartThrottlePermil - target);
        if (stepAmplitude < 80U) {
          stepAmplitude = 80U;
        }
        uint32_t relOvershoot = (static_cast<uint32_t>(overshoot) * 1000UL) / stepAmplitude;
        if (relOvershoot > 2000UL) {
          relOvershoot = 2000UL;
        }
        if (relOvershoot > cal.tuneMaxOvershootRelPermil) {
          cal.tuneMaxOvershootRelPermil = static_cast<uint16_t>(relOvershoot);
        }
      }

      uint16_t absError = static_cast<uint16_t>(abs(errorPermil));
      uint32_t stepElapsed = nowMs - cal.tuneStepStartMs;

      if (stepElapsed >= 900UL) {
        cal.tuneLastSteadyErrorPermil = absError;
      }

      bool settled = false;
      if (stepElapsed >= kTuneMinSettleMs && absError <= kTuneSettleBandPermil) {
        if (cal.tuneStableSamples < 250U) {
          cal.tuneStableSamples++;
        }
      } else {
        cal.tuneStableSamples = 0U;
      }

      if (cal.tuneStableSamples >= 8U) {
        settled = true;
      }

      if (settled || stepElapsed >= kTuneStepTimeoutMs) {
        uint16_t settleMs = static_cast<uint16_t>((stepElapsed > 0xFFFFUL) ? 0xFFFFU : stepElapsed);
        if (settled && settleMs < kTuneMinSettleMs) {
          settleMs = static_cast<uint16_t>(kTuneMinSettleMs);
        }
        if (settleMs > cal.tuneWorstSettleMs) {
          cal.tuneWorstSettleMs = settleMs;
        }

        cal.tuneStableSamples = 0U;
        cal.tuneStep++;
        cal.tuneStepStartMs = nowMs;
        cal.tuneStepStartThrottlePermil = throttlePos;
        cal.tuneIntegral = 0.0f;
        cal.tuneLastError = static_cast<float>(errorPermil);

        if (cal.tuneStep >= 4U) {
          status.autoTuneMaxOvershoot = cal.tuneMaxOvershootPermil;
          status.autoTuneMaxSettleMs = cal.tuneWorstSettleMs;
          status.autoTuneLastSteadyError = cal.tuneLastSteadyErrorPermil;

          if (cal.tunePassIndex == 0U) {
            adjustTuneGainsFromMetrics(cal);
            cal.tunePassIndex = 1U;
            cal.tuneStep = 0U;
            cal.tunePassStartMs = nowMs;
            cal.tuneStepStartMs = nowMs;
            cal.tuneStepStartThrottlePermil = throttlePos;
            cal.tuneMaxThrottlePermil = throttlePos;
            cal.tuneMaxOvershootPermil = 0U;
            cal.tuneMaxOvershootRelPermil = 0U;
            cal.tuneWorstSettleMs = 0U;
            cal.tuneLastSteadyErrorPermil = 0U;
          } else {
            bool accepted = (cal.tuneMaxOvershootPermil <= 320U) &&
                            (cal.tuneMaxOvershootRelPermil <= kTuneMaxRelOvershootPermil) &&
                            (cal.tuneWorstSettleMs <= 2200U) &&
                            (cal.tuneLastSteadyErrorPermil <= 60U);

            if (accepted && cal.tuneSuccessCount < kTuneTrialCount) {
              cal.tuneKpSamples[cal.tuneSuccessCount] = cal.tuneKp_x1000;
              cal.tuneKiSamples[cal.tuneSuccessCount] = cal.tuneKi_x1000;
              cal.tuneKdSamples[cal.tuneSuccessCount] = cal.tuneKd_x1000;
              cal.tuneSuccessCount++;
            }

            cal.tuneTrialIndex++;

            if (cal.tuneTrialIndex < kTuneTrialCount) {
              resetTuneTrialState(cal,
                                  nowMs,
                                  throttlePos,
                                  cal.tuneOrigKp_x1000,
                                  cal.tuneOrigKi_x1000,
                                  cal.tuneOrigKd_x1000);
              status.autoTunePass = static_cast<uint16_t>((cal.tuneTrialIndex * 2U) + 1U);
            } else {
              bool overallAccepted = (cal.tuneSuccessCount >= 2U);

              if (overallAccepted) {
                cfg.kp_x1000 = aggregateMedianOrAverage(cal.tuneKpSamples, cal.tuneSuccessCount);
                cfg.ki_x1000 = aggregateMedianOrAverage(cal.tuneKiSamples, cal.tuneSuccessCount);
                cfg.kd_x1000 = aggregateMedianOrAverage(cal.tuneKdSamples, cal.tuneSuccessCount);
                status.autoTuneResult = DBW_AUTOTUNE_RESULT_PASS;
                result.configChanged = true;
                status.state = (cfg.enabled != 0U) ? DBW_STATE_ARMED : DBW_STATE_IDLE;
                status.faultReason = DBW_FAULT_NONE;
              } else {
                cfg.kp_x1000 = cal.tuneOrigKp_x1000;
                cfg.ki_x1000 = cal.tuneOrigKi_x1000;
                cfg.kd_x1000 = cal.tuneOrigKd_x1000;
                status.autoTuneResult = DBW_AUTOTUNE_RESULT_FAIL;
                status.state = DBW_STATE_FAULT;
                status.faultReason = DBW_FAULT_CALIBRATION;
              }

              if (cal.tuneControlModeForced) {
                dbwSetControlMode(cfg, cal.savedControlMode);
                cal.tuneControlModeForced = false;
                result.configChanged = true;
              }

              cal.active = false;
              status.calStage = 0U;
              status.calProgressPermil = 1000U;
              status.targetPermil = 0U;
              status.motorDuty = 0U;
              zeroMotorOutputs(outputDuty);
              result.outputsChanged = true;
              return result;
            }
          }
        }
      }

      uint32_t tuneElapsedMs = nowMs - cal.stageStartMs;
      constexpr uint32_t kTuneTotalWindowMs = 45000UL;
      if (tuneElapsedMs >= kTuneTotalWindowMs) {
        status.calProgressPermil = 1000U;
      } else {
        status.calProgressPermil = static_cast<uint16_t>((tuneElapsedMs * 1000UL) / kTuneTotalWindowMs);
      }

      result.outputsChanged = true;
      return result;
    }

    if (cal.mode == DBW_CAL_MODE_PEDAL) {
      switch (cal.stage) {
        case 0U:
          captureClosedBaseline(cal.pedalClosedSum, cal.pedalClosedSamples, pedalRaw);
          if (elapsed >= 2000UL) {
            cal.pedalClosedBaseline[0] = static_cast<uint16_t>(cal.pedalClosedSum[0] / (cal.pedalClosedSamples == 0U ? 1U : cal.pedalClosedSamples));
            cal.pedalClosedBaseline[1] = static_cast<uint16_t>(cal.pedalClosedSum[1] / (cal.pedalClosedSamples == 0U ? 1U : cal.pedalClosedSamples));
            cal.stage = 1U;
            cal.stageStartMs = nowMs;
          }
          break;

        case 1U:
          updateExtremeFromBaseline(cal.pedalOpenExtreme, cal.pedalOpenInit, pedalRaw, cal.pedalClosedBaseline);
          if (elapsed >= 4000UL) {
            for (int i = 0; i < 2; i++) {
              cfg.pedalClosedRaw[i] = static_cast<uint16_t>(cal.pedalClosedSum[i] / (cal.pedalClosedSamples == 0U ? 1U : cal.pedalClosedSamples));
              cfg.pedalOpenRaw[i] = cal.pedalOpenExtreme[i];
            }
            refreshCalibratedFlag(cfg);
            cal.active = false;
            status.state = (cfg.calibrated != 0U && cfg.enabled != 0U) ? DBW_STATE_ARMED : DBW_STATE_IDLE;
            status.calStage = 0U;
            status.calProgressPermil = 1000U;
            result.configChanged = true;
            result.outputsChanged = true;
            zeroMotorOutputs(outputDuty);
            return result;
          }
          break;

        default:
          cal.active = false;
          status.state = DBW_STATE_FAULT;
          status.faultReason = DBW_FAULT_CALIBRATION;
          zeroMotorOutputs(outputDuty);
          result.outputsChanged = true;
          return result;
      }
    } else {
      switch (cal.stage) {
        case 0U:
          captureClosedBaseline(cal.throttleClosedSum, cal.throttleClosedSamples, throttleRaw);
          if (elapsed >= 2000UL) {
            cal.throttleClosedBaseline[0] = static_cast<uint16_t>(cal.throttleClosedSum[0] / (cal.throttleClosedSamples == 0U ? 1U : cal.throttleClosedSamples));
            cal.throttleClosedBaseline[1] = static_cast<uint16_t>(cal.throttleClosedSum[1] / (cal.throttleClosedSamples == 0U ? 1U : cal.throttleClosedSamples));
            cal.stage = 1U;
            cal.stageStartMs = nowMs;
            cal.openResponseMs = 0U;
          }
          break;

        case 1U: {
          bool openDirection = (cfg.directionInvert == 0U);
          status.motorDuty = 120U;
          setMotorOutputs(cfg, outputDuty, openDirection, 120U);
          updateExtremeFromBaseline(cal.throttleOpenExtreme, cal.throttleOpenInit, throttleRaw, cal.throttleClosedBaseline);
          if (cal.openResponseMs == 0U) {
            uint16_t response = calculateResponseWindow(throttleRaw, cal.throttleClosedBaseline, elapsed);
            if (response != 0U) {
              cal.openResponseMs = response;
            }
          }
          if (elapsed >= 4000UL) {
            cal.stage = 2U;
            cal.stageStartMs = nowMs;
            cal.closeResponseMs = 0U;
          }
          break;
        }

        case 2U: {
          bool openDirection = (cfg.directionInvert == 0U);
          status.motorDuty = 120U;
          setMotorOutputs(cfg, outputDuty, !openDirection, 120U);
          if (cal.closeResponseMs == 0U) {
            uint16_t response = calculateResponseWindow(throttleRaw, cal.throttleClosedBaseline, elapsed);
            if (response != 0U) {
              cal.closeResponseMs = response;
            }
          }
          if (elapsed >= 4000UL) {
            for (int i = 0; i < 2; i++) {
              cfg.throttleClosedRaw[i] = static_cast<uint16_t>(cal.throttleClosedSum[i] / (cal.throttleClosedSamples == 0U ? 1U : cal.throttleClosedSamples));
              cfg.throttleOpenRaw[i] = cal.throttleOpenExtreme[i];
            }

            if (cal.mode == DBW_CAL_MODE_FULL) {
              uint32_t responseMs = cal.openResponseMs;
              if (cal.closeResponseMs > responseMs) {
                responseMs = cal.closeResponseMs;
              }
              if (responseMs == 0U) {
                responseMs = 250U;
              }
              uint32_t kp = 250000UL / responseMs;
              if (kp < 160UL) {
                kp = 160UL;
              }
              if (kp > 650UL) {
                kp = 650UL;
              }
              cfg.kp_x1000 = static_cast<uint16_t>(kp);
              cfg.ki_x1000 = static_cast<uint16_t>((kp / 12UL) < 5UL ? 5UL : (kp / 12UL));
              cfg.kd_x1000 = static_cast<uint16_t>((responseMs / 8UL) < 5UL ? 5UL : (responseMs / 8UL));
            }

            refreshCalibratedFlag(cfg);
            if (cfg.calibrated != 0U) {
              cfg.enabled = 1U;
            }
            cal.active = false;
            status.state = (cfg.calibrated != 0U && cfg.enabled != 0U) ? DBW_STATE_ARMED : DBW_STATE_IDLE;
            status.calStage = 0U;
            status.calProgressPermil = 1000U;
            result.configChanged = true;
            result.outputsChanged = true;
            zeroMotorOutputs(outputDuty);
            return result;
          }
          break;
        }

        default:
          cal.active = false;
          status.state = DBW_STATE_FAULT;
          status.faultReason = DBW_FAULT_CALIBRATION;
          zeroMotorOutputs(outputDuty);
          result.outputsChanged = true;
          return result;
      }
    }

    result.outputsChanged = true;
    return result;
  }

  if (cfg.enabled == 0U) {
    status.state = DBW_STATE_IDLE;
    status.faultReason = DBW_FAULT_NONE;
    status.calStage = 0U;
    status.calProgressPermil = 0U;
    status.targetPermil = 0U;
    status.motorDuty = 0U;
    status.errorPermil = 0;
    status.pidIntegral = 0.0f;
    status.pidLastError = 0.0f;
    status.pidStateValid = 0U;
    result.outputsChanged = (prevMotorDuty != 0U);
    return result;
  }

  if (!endpointsValid(cfg) || cfg.calibrated == 0U) {
    status.state = DBW_STATE_IDLE;
    status.faultReason = DBW_FAULT_CONFIG;
    status.motorDuty = 0U;
    status.errorPermil = 0;
    status.pidIntegral = 0.0f;
    status.pidLastError = 0.0f;
    status.pidStateValid = 0U;
    zeroMotorOutputs(outputDuty);
    result.outputsChanged = (prevMotorDuty != 0U);
    return result;
  }

  status.state = DBW_STATE_ARMED;
  status.faultReason = DBW_FAULT_NONE;

  if (externalTargetActive && !externalTargetValid) {
    status.state = DBW_STATE_FAULT;
    status.faultReason = DBW_FAULT_RANGE;
    status.motorDuty = 0U;
    status.errorPermil = 0;
    status.pidIntegral = 0.0f;
    status.pidLastError = 0.0f;
    status.pidStateValid = 0U;
    zeroMotorOutputs(outputDuty);
    result.outputsChanged = (prevMotorDuty != 0U);
    return result;
  }

  uint16_t pedalValue = averagePermil(pedal0, pedal1);
  uint16_t pedalTarget = status.manualTargetActive
                           ? status.targetPermil
                           : (externalTargetActive ? externalTargetPermil : pedalValue);
  uint16_t throttlePos = averagePermil(throttle0, throttle1);

  status.pedalPermil = pedalValue;
  status.throttlePermil = throttlePos;

  int32_t error = static_cast<int32_t>(pedalTarget) - static_cast<int32_t>(throttlePos);
  status.errorPermil = static_cast<int16_t>((error < -32768L) ? -32768L : (error > 32767L ? 32767L : error));

  if (abs(status.errorPermil) <= static_cast<int16_t>(cfg.deadbandPermil)) {
    status.targetPermil = pedalTarget;
    status.motorDuty = 0U;
    status.pidStateValid = 1U;
    zeroMotorOutputs(outputDuty);
    result.outputsChanged = (prevMotorDuty != 0U);
    return result;
  }

  float dt = 0.01f;
  if (prevUpdateMs != 0U && nowMs > prevUpdateMs) {
    dt = static_cast<float>(nowMs - prevUpdateMs) / 1000.0f;
    if (dt < 0.002f) {
      dt = 0.002f;
    } else if (dt > 0.05f) {
      dt = 0.05f;
    }
  }

  if (status.pidStateValid == 0U || prevUpdateMs == 0U || status.state != DBW_STATE_ARMED) {
    status.pidIntegral = 0.0f;
    status.pidLastError = static_cast<float>(status.errorPermil);
    status.pidStateValid = 1U;
  }

  float kp = static_cast<float>(cfg.kp_x1000) / 1000.0f;
  float ki = static_cast<float>(cfg.ki_x1000) / 1000.0f;
  float kd = static_cast<float>(cfg.kd_x1000) / 1000.0f;

  status.pidIntegral += static_cast<float>(status.errorPermil) * dt;
  if (status.pidIntegral > 2000.0f) {
    status.pidIntegral = 2000.0f;
  } else if (status.pidIntegral < -2000.0f) {
    status.pidIntegral = -2000.0f;
  }

  float derivative = (static_cast<float>(status.errorPermil) - status.pidLastError) / dt;
  status.pidLastError = static_cast<float>(status.errorPermil);

  float command = (kp * static_cast<float>(status.errorPermil)) + (ki * status.pidIntegral) + (kd * derivative);
  bool openDirection = (command >= 0.0f);
  if (cfg.directionInvert != 0U) {
    openDirection = !openDirection;
  }

  uint16_t duty = static_cast<uint16_t>(fabsf(command));
  if (duty > cfg.maxDuty) {
    duty = cfg.maxDuty;
  }
  if (duty > 0U && duty < 35U) {
    duty = 35U;
  }

  status.targetPermil = pedalTarget;
  status.motorDuty = duty;
  setMotorOutputs(cfg, outputDuty, openDirection, duty);
  result.outputsChanged = true;
  return result;
}