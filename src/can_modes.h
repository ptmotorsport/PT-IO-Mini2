#pragma once

#include <Arduino.h>
#include <Arduino_CAN.h>

constexpr uint8_t CAN_MODE_COUNT = 16;

enum CanModeId : uint8_t {
  CAN_MODE_PT_DEFAULT1 = 0,
  CAN_MODE_HALTECH_IO12A = 1,
  CAN_MODE_HALTECH_IO12B = 2,
  CAN_MODE_HALTECH_IO12AB = 3,
  CAN_MODE_HALTECH_IO16A = 4,
  CAN_MODE_HALTECH_IO16B = 5,
  CAN_MODE_ECUMASTER_CANSWB_V3 = 6,
  CAN_MODE_MOTEC_E888 = 7,
  CAN_MODE_EMTRON = 8,
  CAN_MODE_RESERVED_9 = 9,
  CAN_MODE_RESERVED_10 = 10,
  CAN_MODE_RESERVED_11 = 11,
  CAN_MODE_RESERVED_12 = 12,
  CAN_MODE_RESERVED_13 = 13,
  CAN_MODE_RESERVED_14 = 14,
  CAN_MODE_RESERVED_15 = 15,
};

struct ModeTxFrame {
  uint32_t id;
  uint8_t data[8];
  uint8_t len;
};

struct Mode0Status {
  bool canInitOk;
  bool outputsInSafeState;
  uint16_t canRxCount;
  uint16_t canTxCount;
  uint16_t canTxFail;
  uint8_t canMode;
  uint32_t lastCanRxMs;
  uint16_t rxTimeoutMs;
  uint32_t nowMs;
};

uint16_t mode0ScaleAnalogTo12Bit(uint16_t analogRaw14);

bool mode0HandleRx(const CanMsg &msg,
                   uint16_t rxBaseId,
                   uint16_t defaultPwmFreqHz,
                   uint16_t outputFreq[8],
                   uint8_t outputDuty[8],
                   uint8_t &safeMask,
                   uint8_t &activeMask,
                   bool &maskChanged);

void mode0BuildTxAnalogFrames(uint16_t txBaseId,
                              const uint16_t analogRaw14[8],
                              ModeTxFrame &frame0,
                              ModeTxFrame &frame1);

void mode0BuildTxStateFrame(uint16_t txBaseId,
                            uint8_t digitalInMask,
                            uint8_t analogStateMask,
                            uint8_t digitalOutMask,
                            uint8_t safeMask,
                            uint8_t activeMask,
                            uint8_t fwVersion,
                            ModeTxFrame &frame);

void mode0BuildTxDiPairFrame(uint32_t baseId,
                             uint32_t timerFreq0,
                             uint32_t period0,
                             uint32_t high0,
                             bool hasPeriod0,
                             uint32_t timerFreq1,
                             uint32_t period1,
                             uint32_t high1,
                             bool hasPeriod1,
                             ModeTxFrame &frame);

void mode0BuildTxStatusFrame(uint16_t txBaseId,
                             const Mode0Status &status,
                             ModeTxFrame &frame);

const char *canModeName(uint8_t mode);

bool canModeHandleRx(uint8_t mode,
                     const CanMsg &msg,
                     uint16_t rxBaseId,
                     uint16_t defaultPwmFreqHz,
                     uint16_t outputFreq[8],
                     uint8_t outputDuty[8],
                     uint8_t &safeMask,
                     uint8_t &activeMask,
                     bool &maskChanged);

void canModeBuildTxAnalogFrames(uint8_t mode,
                                uint16_t txBaseId,
                                const uint16_t analogRaw14[8],
                                ModeTxFrame &frame0,
                                ModeTxFrame &frame1);

void canModeBuildTxStateFrame(uint8_t mode,
                              uint16_t txBaseId,
                              uint8_t digitalInMask,
                              uint8_t analogStateMask,
                              uint8_t digitalOutMask,
                              uint8_t safeMask,
                              uint8_t activeMask,
                              uint8_t fwVersion,
                              ModeTxFrame &frame);

void canModeBuildTxDiPairFrame(uint8_t mode,
                               uint32_t baseId,
                               uint32_t timerFreq0,
                               uint32_t period0,
                               uint32_t high0,
                               bool hasPeriod0,
                               uint32_t timerFreq1,
                               uint32_t period1,
                               uint32_t high1,
                               bool hasPeriod1,
                               ModeTxFrame &frame);

void canModeBuildTxStatusFrame(uint8_t mode,
                               uint16_t txBaseId,
                               const Mode0Status &status,
                               ModeTxFrame &frame);