#pragma once

#include <Arduino.h>

constexpr uint8_t DBW_STATE_IDLE = 0;
constexpr uint8_t DBW_STATE_ARMED = 1;
constexpr uint8_t DBW_STATE_CALIBRATING = 2;
constexpr uint8_t DBW_STATE_FAULT = 3;

constexpr uint8_t DBW_FAULT_NONE = 0;
constexpr uint8_t DBW_FAULT_SENSOR = 1;
constexpr uint8_t DBW_FAULT_CALIBRATION = 2;
constexpr uint8_t DBW_FAULT_RANGE = 3;
constexpr uint8_t DBW_FAULT_CONFIG = 4;

constexpr uint8_t DBW_CONTROL_APS = 0;
constexpr uint8_t DBW_CONTROL_DIGITAL_IN = 1;
constexpr uint8_t DBW_CONTROL_CAN = 2;

struct DbwConfig {
  uint16_t pedalClosedRaw[2];
  uint16_t pedalOpenRaw[2];
  uint16_t throttleClosedRaw[2];
  uint16_t throttleOpenRaw[2];
  uint16_t kp_x1000;
  uint16_t ki_x1000;
  uint16_t kd_x1000;
  uint16_t pwmFreqHz;
  uint16_t deadbandPermil;
  uint16_t sensorMismatchPermil;
  uint16_t maxDuty;
  uint8_t enabled;
  uint8_t directionInvert;
  uint8_t calibrated;
  uint8_t reserved;
};

struct DbwStatus {
  uint8_t state;
  uint8_t faultReason;
  uint8_t calStage;
  uint8_t manualTargetActive;
  uint8_t autoTuneResult;
  uint16_t pedalPermil;
  uint16_t throttlePermil;
  uint16_t targetPermil;
  uint16_t motorDuty;
  uint16_t autoTunePass;
  uint16_t autoTuneMaxOvershoot;
  uint16_t autoTuneMaxSettleMs;
  uint16_t autoTuneLastSteadyError;
  int16_t errorPermil;
  uint16_t calProgressPermil;
  uint32_t lastUpdateMs;
  float pidIntegral;
  float pidLastError;
  uint8_t pidStateValid;
};

struct DbwServiceResult {
  bool outputsChanged;
  bool configChanged;
};

void dbwSetDefaults(DbwConfig &cfg);
void dbwResetRuntime(DbwStatus &status);
void dbwBeginCalibration(DbwStatus &status, uint32_t nowMs);
void dbwBeginPedalCalibration(DbwStatus &status, uint32_t nowMs);
void dbwBeginThrottleCalibration(DbwStatus &status, uint32_t nowMs);
void dbwBeginAutoTune(DbwStatus &status, uint32_t nowMs);
void dbwAbortCalibration(DbwStatus &status, DbwConfig *cfg = nullptr);
void dbwSetManualTarget(DbwStatus &status, bool enabled, uint16_t targetPermil = 0);
void dbwSetEnabled(DbwConfig &cfg, bool enabled);
void dbwSetMotorInvert(DbwConfig &cfg, bool invert);
void dbwSetInhInverted(DbwConfig &cfg, bool invert);
bool dbwGetInhInverted(const DbwConfig &cfg);
void dbwSetControlMode(DbwConfig &cfg, uint8_t mode);
uint8_t dbwGetControlMode(const DbwConfig &cfg);
const char *dbwControlModeName(uint8_t mode);
void dbwSetDigitalInputChannel(DbwConfig &cfg, uint8_t channel);
uint8_t dbwGetDigitalInputChannel(const DbwConfig &cfg);
const char *dbwStateName(uint8_t state);
const char *dbwFaultName(uint8_t fault);
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
                            uint16_t externalFreqHz);