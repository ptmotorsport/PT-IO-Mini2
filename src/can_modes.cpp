#include "can_modes.h"

static const char *CAN_MODE_NAMES[CAN_MODE_COUNT] = {
  "PT_Default1",
  "Haltech IO12A",
  "Haltech IO12B",
  "Haltech IO12A&B",
  "Haltech IO16A",
  "Haltech IO16B",
  "ECU Master CANSWB v3",
  "Motec E888",
  "Emtron",
  "reserved",
  "reserved",
  "reserved",
  "reserved",
  "reserved",
  "reserved",
  "reserved",
};

static uint16_t clampU16(uint32_t value) {
  return static_cast<uint16_t>(value > 65535UL ? 65535UL : value);
}

static uint8_t clampU8(uint32_t value) {
  return static_cast<uint8_t>(value > 255UL ? 255UL : value);
}

static constexpr uint32_t HALTECH_DPO_CONTROL_ID = 0x6A8;
static constexpr uint32_t HALTECH_DPO_CONFIG_ID = 0x6A9;
static constexpr uint32_t HALTECH_AVI_STATUS_ID = 0x330;
static constexpr uint32_t HALTECH_SPI_STATUS_ID_1 = 0x332;
static constexpr uint32_t HALTECH_SPI_STATUS_ID_2 = 0x333;
static constexpr uint8_t HALTECH_MUX_ID_DPO = 2;

static constexpr uint32_t IO12_AVI_TX_ID_A = 0x2C0;
static constexpr uint32_t IO12_DPI_TX_ID_1_A = 0x2C2;
static constexpr uint32_t IO12_DPI_TX_ID_2_A = 0x2C4;
static constexpr uint32_t IO12_DPO_RX_ID_1_A = 0x2D0;
static constexpr uint32_t IO12_DPO_RX_ID_2_A = 0x2D2;

struct Io12TxState {
  uint8_t digitalInMask;
  uint8_t selectedBank;
  uint8_t diCallIndex;
  bool mode3Toggle;
};

static Io12TxState io12TxState = {0, 0, 0, false};

static bool isHaltechIo16Mode(uint8_t mode) {
  return mode == CAN_MODE_HALTECH_IO16A ||
         mode == CAN_MODE_HALTECH_IO16B;
}

static bool isHaltechIo12Mode(uint8_t mode) {
  return mode == CAN_MODE_HALTECH_IO12A ||
         mode == CAN_MODE_HALTECH_IO12B ||
         mode == CAN_MODE_HALTECH_IO12AB;
}

static uint32_t io12IdShiftForModeBank(uint8_t mode, uint8_t bank) {
  if (mode == CAN_MODE_HALTECH_IO12B) {
    return 1U;
  }
  if (mode == CAN_MODE_HALTECH_IO12AB) {
    return (bank == 0U) ? 0U : 1U;
  }
  return 0U;
}

static uint8_t io12BankStartForModeBank(uint8_t mode, uint8_t bank) {
  if (mode == CAN_MODE_HALTECH_IO12AB) {
    return (bank == 0U) ? 0U : 4U;
  }
  return 0U;
}

static uint8_t io12MapRawOutToDuty255(uint8_t rawOut) {
  if (rawOut == 0U) {
    return 0U;
  }
  if (rawOut >= 250U) {
    return 255U;
  }
  return clampU8((static_cast<uint32_t>(rawOut) * 255UL + 124UL) / 249UL);
}

static uint32_t io12PeriodScaledFromCapture(uint32_t timerFreqHz, uint32_t periodCounts, bool hasPeriod) {
  if (!hasPeriod || timerFreqHz == 0U || periodCounts == 0U) {
    return 0U;
  }
  uint32_t periodUs = static_cast<uint32_t>((static_cast<uint64_t>(periodCounts) * 1000000ULL) / timerFreqHz);
  uint32_t periodScaled = periodUs / 10UL;
  if (periodScaled > 0xFFFFFU) {
    periodScaled = 0xFFFFFU;
  }
  return periodScaled;
}

static uint8_t io12DutyByteFromCapture(uint32_t periodCounts, uint32_t highCounts, bool hasPeriod) {
  if (!hasPeriod || periodCounts == 0U) {
    return 0U;
  }
  uint32_t dutyPct = (highCounts * 100UL) / periodCounts;
  if (dutyPct > 100UL) {
    dutyPct = 100UL;
  }
  return clampU8((dutyPct * 250UL + 50UL) / 100UL);
}

static bool io12DpiStateFromMask(uint8_t mask, uint8_t channelIndex) {
  if (channelIndex >= 8U) {
    return false;
  }
  bool pinLevelHigh = ((mask >> channelIndex) & 0x01U) != 0;
  return !pinLevelHigh;
}

static void io12BuildDpiFrame1(uint32_t txId,
                               uint8_t channelStart,
                               uint8_t digitalInMask,
                               uint32_t timerFreq0,
                               uint32_t period0,
                               uint32_t high0,
                               bool hasPeriod0,
                               ModeTxFrame &frame) {
  frame.id = txId;
  frame.len = 8;
  memset(frame.data, 0, sizeof(frame.data));

  uint8_t dutyByte = io12DutyByteFromCapture(period0, high0, hasPeriod0);
  uint32_t periodScaled = io12PeriodScaledFromCapture(timerFreq0, period0, hasPeriod0);
  bool dpi1State = io12DpiStateFromMask(digitalInMask, channelStart);
  bool dpi2State = io12DpiStateFromMask(digitalInMask, static_cast<uint8_t>(channelStart + 1U));

  frame.data[0] = dutyByte;
  frame.data[1] |= static_cast<uint8_t>((dpi1State ? 1U : 0U) << 5);
  frame.data[1] |= static_cast<uint8_t>((periodScaled >> 16) & 0xF0U);
  frame.data[2] = static_cast<uint8_t>((periodScaled >> 8) & 0xFFU);
  frame.data[3] = static_cast<uint8_t>(periodScaled & 0xFFU);
  frame.data[5] |= static_cast<uint8_t>((dpi2State ? 1U : 0U) << 5);
}

static void io12BuildDpiFrame2(uint32_t txId,
                               uint8_t channelStart,
                               uint8_t digitalInMask,
                               ModeTxFrame &frame) {
  frame.id = txId;
  frame.len = 8;
  memset(frame.data, 0, sizeof(frame.data));

  bool dpi3State = io12DpiStateFromMask(digitalInMask, static_cast<uint8_t>(channelStart + 2U));
  bool dpi4State = io12DpiStateFromMask(digitalInMask, static_cast<uint8_t>(channelStart + 3U));
  frame.data[1] |= static_cast<uint8_t>((dpi3State ? 1U : 0U) << 5);
  frame.data[5] |= static_cast<uint8_t>((dpi4State ? 1U : 0U) << 5);
}

static uint16_t scaleAnalog14ToMv5000(uint16_t analogRaw14) {
  uint32_t mv = (static_cast<uint32_t>(analogRaw14) * 5000UL + 8191UL) / 16383UL;
  if (mv > 5000UL) {
    mv = 5000UL;
  }
  return static_cast<uint16_t>(mv);
}

static uint8_t mapDuty1000To255(uint16_t dutyRaw1000) {
  if (dutyRaw1000 > 1000U) {
    dutyRaw1000 = 1000U;
  }
  return static_cast<uint8_t>((static_cast<uint32_t>(dutyRaw1000) * 255UL + 500UL) / 1000UL);
}

static uint16_t haltechRawPeriodToHz(uint16_t rawPeriod) {
  if (rawPeriod == 0U) {
    return 0U;
  }

  // Haltech IO mode uses an odd scaled period/frequency field rather than direct Hz.
  // The reference sketch scales frequency as quarter-Hz for one SPI status path.
  // For output PWM control we decode this field to a usable Hz approximation.
  // Current mapping assumes 4 raw counts = 1 Hz.
  uint32_t hz = static_cast<uint32_t>(rawPeriod) / 4UL;
  if (hz == 0UL) {
    hz = 1UL;
  }
  return static_cast<uint16_t>(hz);
}

static void packHaltechStateAndMv(uint16_t mv, bool state, uint8_t &byteMsbState, uint8_t &byteLsb) {
  byteMsbState = static_cast<uint8_t>(((state ? 1U : 0U) << 7) | ((mv >> 8) & 0x7FU));
  byteLsb = static_cast<uint8_t>(mv & 0xFFU);
}

uint16_t mode0ScaleAnalogTo12Bit(uint16_t analogRaw14) {
  uint32_t scaled = (static_cast<uint32_t>(analogRaw14) * 4095UL + 8191UL) / 16383UL;
  if (scaled > 4095UL) {
    scaled = 4095UL;
  }
  return static_cast<uint16_t>(scaled);
}

bool mode0HandleRx(const CanMsg &msg,
                   uint16_t rxBaseId,
                   uint16_t defaultPwmFreqHz,
                   uint16_t outputFreq[8],
                   uint8_t outputDuty[8],
                   uint8_t &safeMask,
                   uint8_t &activeMask,
                   bool &maskChanged,
                   uint8_t &inputPullupMask,
                   bool &pullupChanged) {
  maskChanged = false;
  pullupChanged = false;

  uint32_t id = msg.id;
  if (id == (rxBaseId + 4U)) {
    if (msg.data_length < 1) {
      return false;
    }
    uint8_t newMask = msg.data[0];
    pullupChanged = (newMask != inputPullupMask);
    inputPullupMask = newMask;
    return true;
  }

  if (id < rxBaseId || id > (rxBaseId + 3U)) {
    return false;
  }

  if (msg.data_length < 8) {
    return false;
  }

  uint8_t idxPair = static_cast<uint8_t>(id - rxBaseId); // 0..3
  uint8_t outA = static_cast<uint8_t>(idxPair * 2U);
  uint8_t outB = static_cast<uint8_t>(outA + 1U);

  uint16_t freqA = static_cast<uint16_t>(msg.data[0] | (msg.data[1] << 8));
  uint8_t dutyA = msg.data[2];
  uint8_t flagsA = msg.data[3];

  uint16_t freqB = static_cast<uint16_t>(msg.data[4] | (msg.data[5] << 8));
  uint8_t dutyB = msg.data[6];
  uint8_t flagsB = msg.data[7];

  // Use defaultPwmFreqHz when the sender signals 0 (don't-care), then clamp
  // to the same 50–10000 Hz range enforced on EEPROM load.  An out-of-range
  // value from a mis-configured sender must not reach the GPT timer init.
  if (freqA == 0U) freqA = defaultPwmFreqHz;
  if (freqB == 0U) freqB = defaultPwmFreqHz;
  if (freqA <   50U) freqA =   50U;
  if (freqA > 10000U) freqA = 10000U;
  if (freqB <   50U) freqB =   50U;
  if (freqB > 10000U) freqB = 10000U;

  outputFreq[outA] = freqA;
  outputFreq[outB] = freqB;
  outputDuty[outA] = dutyA;
  outputDuty[outB] = dutyB;

  bool safeA = (flagsA & 0x01U) != 0;
  bool activeA = (flagsA & 0x02U) != 0;
  bool safeB = (flagsB & 0x01U) != 0;
  bool activeB = (flagsB & 0x02U) != 0;

  uint8_t newSafeMask = safeMask;
  uint8_t newActiveMask = activeMask;

  newSafeMask = static_cast<uint8_t>((newSafeMask & ~(1U << outA)) | (safeA ? (1U << outA) : 0U));
  newSafeMask = static_cast<uint8_t>((newSafeMask & ~(1U << outB)) | (safeB ? (1U << outB) : 0U));
  newActiveMask = static_cast<uint8_t>((newActiveMask & ~(1U << outA)) | (activeA ? (1U << outA) : 0U));
  newActiveMask = static_cast<uint8_t>((newActiveMask & ~(1U << outB)) | (activeB ? (1U << outB) : 0U));

  maskChanged = (newSafeMask != safeMask) || (newActiveMask != activeMask);
  safeMask = newSafeMask;
  activeMask = newActiveMask;

  return true;
}

void mode0BuildTxAnalogFrames(uint16_t txBaseId,
                              const uint16_t analogRaw14[8],
                              ModeTxFrame &frame0,
                              ModeTxFrame &frame1) {
  frame0.id = txBaseId;
  frame0.len = 8;
  memset(frame0.data, 0, sizeof(frame0.data));
  for (int i = 0; i < 4; i++) {
    uint16_t v = mode0ScaleAnalogTo12Bit(analogRaw14[i]);
    frame0.data[i * 2] = static_cast<uint8_t>(v & 0xFF);
    frame0.data[i * 2 + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  }

  frame1.id = txBaseId + 1;
  frame1.len = 8;
  memset(frame1.data, 0, sizeof(frame1.data));
  for (int i = 0; i < 4; i++) {
    uint16_t v = mode0ScaleAnalogTo12Bit(analogRaw14[i + 4]);
    frame1.data[i * 2] = static_cast<uint8_t>(v & 0xFF);
    frame1.data[i * 2 + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  }
}

void mode0BuildTxStateFrame(uint16_t txBaseId,
                            uint8_t digitalInMask,
                            uint8_t analogStateMask,
                            uint8_t digitalOutMask,
                            uint8_t safeMask,
                            uint8_t activeMask,
                            uint8_t fwVersion,
                            ModeTxFrame &frame) {
  frame.id = txBaseId + 2;
  frame.len = 8;
  memset(frame.data, 0, sizeof(frame.data));
  frame.data[0] = digitalInMask;
  frame.data[1] = analogStateMask;
  frame.data[2] = digitalOutMask;
  frame.data[3] = safeMask;
  frame.data[4] = activeMask;
  frame.data[7] = fwVersion;
}

void mode0BuildTxDiPairFrame(uint32_t baseId,
                             uint32_t timerFreq0,
                             uint32_t period0,
                             uint32_t high0,
                             bool hasPeriod0,
                             uint32_t timerFreq1,
                             uint32_t period1,
                             uint32_t high1,
                             bool hasPeriod1,
                             ModeTxFrame &frame) {
  uint16_t freq0 = 0;
  uint16_t freq1 = 0;
  uint8_t duty0 = 0;
  uint8_t duty1 = 0;

  if (hasPeriod0 && period0 > 0 && timerFreq0 > 0) {
    freq0 = clampU16(timerFreq0 / period0);
    duty0 = static_cast<uint8_t>((high0 > 0) ? ((high0 * 255UL) / period0 > 255UL ? 255UL : (high0 * 255UL) / period0) : 0UL);
  }
  if (hasPeriod1 && period1 > 0 && timerFreq1 > 0) {
    freq1 = clampU16(timerFreq1 / period1);
    duty1 = static_cast<uint8_t>((high1 > 0) ? ((high1 * 255UL) / period1 > 255UL ? 255UL : (high1 * 255UL) / period1) : 0UL);
  }

  frame.id = baseId;
  frame.len = 8;
  memset(frame.data, 0, sizeof(frame.data));
  frame.data[0] = static_cast<uint8_t>(freq0 & 0xFF);
  frame.data[1] = static_cast<uint8_t>((freq0 >> 8) & 0xFF);
  frame.data[2] = duty0;
  frame.data[4] = static_cast<uint8_t>(freq1 & 0xFF);
  frame.data[5] = static_cast<uint8_t>((freq1 >> 8) & 0xFF);
  frame.data[6] = duty1;
}

void mode0BuildTxStatusFrame(uint16_t txBaseId,
                             const Mode0Status &status,
                             ModeTxFrame &frame) {
  bool canSilent = (status.lastCanRxMs == 0)
                     ? (status.nowMs > status.rxTimeoutMs)
                     : ((status.nowMs - status.lastCanRxMs) > status.rxTimeoutMs);

  frame.id = txBaseId + 7;
  frame.len = 8;
  memset(frame.data, 0, sizeof(frame.data));
  frame.data[0] = static_cast<uint8_t>((status.canInitOk ? 0x01 : 0x00) |
                                       (canSilent ? 0x02 : 0x00) |
                                       (status.outputsInSafeState ? 0x04 : 0x00));
  frame.data[1] = static_cast<uint8_t>(status.canRxCount & 0xFF);
  frame.data[2] = static_cast<uint8_t>((status.canRxCount >> 8) & 0xFF);
  frame.data[3] = static_cast<uint8_t>(status.canTxCount & 0xFF);
  frame.data[4] = static_cast<uint8_t>((status.canTxCount >> 8) & 0xFF);
  frame.data[5] = static_cast<uint8_t>(status.canTxFail & 0xFF);
  frame.data[6] = static_cast<uint8_t>((status.canTxFail >> 8) & 0xFF);
  frame.data[7] = status.canMode;
}

const char *canModeName(uint8_t mode) {
  if (mode < CAN_MODE_COUNT) {
    return CAN_MODE_NAMES[mode];
  }
  return "invalid";
}

static bool modeStubHandleRx(const CanMsg &msg,
                             uint16_t rxBaseId,
                             uint16_t defaultPwmFreqHz,
                             uint16_t outputFreq[8],
                             uint8_t outputDuty[8],
                             uint8_t &safeMask,
                             uint8_t &activeMask,
             bool &maskChanged,
             uint8_t &inputPullupMask,
             bool &pullupChanged) {
  return mode0HandleRx(msg,
                       rxBaseId,
                       defaultPwmFreqHz,
                       outputFreq,
                       outputDuty,
                       safeMask,
                       activeMask,
           maskChanged,
           inputPullupMask,
           pullupChanged);
}

static void modeStubBuildTxAnalogFrames(uint16_t txBaseId,
                                        const uint16_t analogRaw14[8],
                                        ModeTxFrame &frame0,
                                        ModeTxFrame &frame1) {
  mode0BuildTxAnalogFrames(txBaseId, analogRaw14, frame0, frame1);
}

static void modeStubBuildTxStateFrame(uint16_t txBaseId,
                                      uint8_t digitalInMask,
                                      uint8_t analogStateMask,
                                      uint8_t digitalOutMask,
                                      uint8_t safeMask,
                                      uint8_t activeMask,
                                      uint8_t fwVersion,
                                      ModeTxFrame &frame) {
  mode0BuildTxStateFrame(txBaseId,
                         digitalInMask,
                         analogStateMask,
                         digitalOutMask,
                         safeMask,
                         activeMask,
                         fwVersion,
                         frame);
}

static void modeStubBuildTxDiPairFrame(uint32_t baseId,
                                       uint32_t timerFreq0,
                                       uint32_t period0,
                                       uint32_t high0,
                                       bool hasPeriod0,
                                       uint32_t timerFreq1,
                                       uint32_t period1,
                                       uint32_t high1,
                                       bool hasPeriod1,
                                       ModeTxFrame &frame) {
  mode0BuildTxDiPairFrame(baseId,
                          timerFreq0,
                          period0,
                          high0,
                          hasPeriod0,
                          timerFreq1,
                          period1,
                          high1,
                          hasPeriod1,
                          frame);
}

static void modeStubBuildTxStatusFrame(uint16_t txBaseId,
                                       const Mode0Status &status,
                                       ModeTxFrame &frame) {
  mode0BuildTxStatusFrame(txBaseId, status, frame);
}

static bool haltechIo16HandleRx(uint8_t mode,
                            const CanMsg &msg,
                            uint16_t defaultPwmFreqHz,
                            uint16_t outputFreq[8],
                            uint8_t outputDuty[8],
                            uint8_t &safeMask,
                            uint8_t &activeMask,
                            bool &maskChanged,
                            bool &pullupChanged) {
  maskChanged = false;
  pullupChanged = false;

  uint8_t bankStart = (mode == CAN_MODE_HALTECH_IO16B) ? 4 : 0;

  if (msg.id == HALTECH_DPO_CONTROL_ID) {
    if (msg.data_length < 5) {
      return false;
    }

    uint8_t muxId = static_cast<uint8_t>((msg.data[0] >> 5) & 0x07);
    uint8_t muxIndex = static_cast<uint8_t>(msg.data[0] & 0x0F);
    if (muxId != HALTECH_MUX_ID_DPO || muxIndex >= 4) {
      return false;
    }

    uint16_t dutyRaw = static_cast<uint16_t>((msg.data[1] << 8) | msg.data[2]);
    uint16_t periodRaw = static_cast<uint16_t>((msg.data[3] << 8) | msg.data[4]);
    uint16_t hz = haltechRawPeriodToHz(periodRaw);
    if (hz == 0U) {
      hz = defaultPwmFreqHz;
    }
    uint8_t duty255 = mapDuty1000To255(dutyRaw);

    uint8_t idx = static_cast<uint8_t>(bankStart + muxIndex);
    if (idx < 8) {
      outputDuty[idx] = duty255;
      outputFreq[idx] = hz;
    }
    return true;
  }

  if (msg.id == HALTECH_DPO_CONFIG_ID) {
    if (msg.data_length < 2) {
      return false;
    }

    uint8_t muxId = static_cast<uint8_t>((msg.data[0] >> 5) & 0x07);
    uint8_t muxIndex = static_cast<uint8_t>(msg.data[0] & 0x0F);
    if (muxId != HALTECH_MUX_ID_DPO || muxIndex >= 4) {
      return false;
    }

    uint8_t safeRaw = static_cast<uint8_t>((msg.data[1] >> 1) & 0x03);
    bool safeOn = (safeRaw == 1U) || (safeRaw == 2U);
    bool activeHigh = (msg.data[1] & 0x01U) != 0;

    auto applyFlags = [&](uint8_t channel) {
      if (channel >= 8) {
        return;
      }
      uint8_t mask = static_cast<uint8_t>(1U << channel);
      uint8_t newSafe = static_cast<uint8_t>((safeMask & ~mask) | (safeOn ? mask : 0U));
      uint8_t newActive = static_cast<uint8_t>((activeMask & ~mask) | (activeHigh ? mask : 0U));
      if (newSafe != safeMask || newActive != activeMask) {
        maskChanged = true;
      }
      safeMask = newSafe;
      activeMask = newActive;
    };

    applyFlags(static_cast<uint8_t>(bankStart + muxIndex));

    return true;
  }

  return false;
}

static void haltechIo16BuildTxAnalogFrames(uint8_t mode,
                                           const uint16_t analogRaw14[8],
                                           ModeTxFrame &frame0,
                                           ModeTxFrame &frame1) {
  uint8_t bankStart = (mode == CAN_MODE_HALTECH_IO16B) ? 4 : 0;

  frame0.id = HALTECH_AVI_STATUS_ID;
  frame0.len = 8;
  memset(frame0.data, 0, sizeof(frame0.data));

  frame1.id = HALTECH_SPI_STATUS_ID_1;
  frame1.len = 8;
  memset(frame1.data, 0, sizeof(frame1.data));

  for (int i = 0; i < 4; i++) {
    uint16_t mv = scaleAnalog14ToMv5000(analogRaw14[bankStart + i]);
    bool state = mv > 100U;

    packHaltechStateAndMv(mv, state, frame0.data[i * 2], frame0.data[i * 2 + 1]);
    packHaltechStateAndMv(mv, state, frame1.data[i * 2], frame1.data[i * 2 + 1]);
  }
}

static bool haltechIo12HandleRx(uint8_t mode,
                                const CanMsg &msg,
                                uint16_t outputFreq[8],
                                uint8_t outputDuty[8],
                                uint8_t &safeMask,
                                uint8_t &activeMask,
                                bool &maskChanged,
                                bool &pullupChanged) {
  maskChanged = false;
  pullupChanged = false;

  auto applyFrame = [&](uint8_t channelStart) {
    if (msg.data_length < 6) {
      return false;
    }

    uint8_t rawOutA = msg.data[0];
    uint8_t flagsA = msg.data[1];
    uint8_t rawOutB = msg.data[4];
    uint8_t flagsB = msg.data[5];

    auto applyOne = [&](uint8_t channel, uint8_t rawOut, uint8_t flags) {
      if (channel >= 8U) {
        return;
      }

      outputDuty[channel] = io12MapRawOutToDuty255(rawOut);

      bool safeOn = (flags & 0x10U) != 0;
      bool activeStateRaw = (flags & 0x20U) != 0; // IO12: true means active LOW
      bool activeHigh = !activeStateRaw;

      uint8_t bitMask = static_cast<uint8_t>(1U << channel);
      uint8_t newSafe = static_cast<uint8_t>((safeMask & ~bitMask) | (safeOn ? bitMask : 0U));
      uint8_t newActive = static_cast<uint8_t>((activeMask & ~bitMask) | (activeHigh ? bitMask : 0U));
      if (newSafe != safeMask || newActive != activeMask) {
        maskChanged = true;
      }
      safeMask = newSafe;
      activeMask = newActive;

      if (outputFreq[channel] == 0U) {
        outputFreq[channel] = 300U;
      }
    };

    applyOne(channelStart, rawOutA, flagsA);
    applyOne(static_cast<uint8_t>(channelStart + 1U), rawOutB, flagsB);
    return true;
  };

  uint32_t shiftA = io12IdShiftForModeBank(mode, 0);
  if (msg.id == IO12_DPO_RX_ID_1_A + shiftA) {
    return applyFrame(io12BankStartForModeBank(mode, 0));
  }
  if (msg.id == IO12_DPO_RX_ID_2_A + shiftA) {
    return applyFrame(static_cast<uint8_t>(io12BankStartForModeBank(mode, 0) + 2U));
  }

  if (mode == CAN_MODE_HALTECH_IO12AB) {
    uint32_t shiftB = io12IdShiftForModeBank(mode, 1);
    if (msg.id == IO12_DPO_RX_ID_1_A + shiftB) {
      return applyFrame(io12BankStartForModeBank(mode, 1));
    }
    if (msg.id == IO12_DPO_RX_ID_2_A + shiftB) {
      return applyFrame(static_cast<uint8_t>(io12BankStartForModeBank(mode, 1) + 2U));
    }
  }

  return false;
}

static void haltechIo12BuildTxAnalogFrame(uint8_t mode,
                                          const uint16_t analogRaw14[8],
                                          ModeTxFrame &frame) {
  uint8_t bank = 0;
  if (mode == CAN_MODE_HALTECH_IO12AB) {
    bank = io12TxState.mode3Toggle ? 1U : 0U;
    io12TxState.mode3Toggle = !io12TxState.mode3Toggle;
  }
  io12TxState.selectedBank = bank;
  io12TxState.diCallIndex = 0;

  uint32_t shift = io12IdShiftForModeBank(mode, bank);
  uint8_t channelStart = io12BankStartForModeBank(mode, bank);

  frame.id = IO12_AVI_TX_ID_A + shift;
  frame.len = 8;
  memset(frame.data, 0, sizeof(frame.data));

  for (int i = 0; i < 4; i++) {
    uint16_t v = mode0ScaleAnalogTo12Bit(analogRaw14[channelStart + i]);
    frame.data[i * 2] = static_cast<uint8_t>((v >> 8) & 0xFFU);
    frame.data[i * 2 + 1] = static_cast<uint8_t>(v & 0xFFU);
  }
}

bool canModeHandleRx(uint8_t mode,
                     const CanMsg &msg,
                     uint16_t rxBaseId,
                     uint16_t defaultPwmFreqHz,
                     uint16_t outputFreq[8],
                     uint8_t outputDuty[8],
                     uint8_t &safeMask,
                     uint8_t &activeMask,
                     bool &maskChanged,
                     uint8_t &inputPullupMask,
                     bool &pullupChanged) {
  switch (mode) {
    case CAN_MODE_PT_DEFAULT1:
      return mode0HandleRx(msg,
                           rxBaseId,
                           defaultPwmFreqHz,
                           outputFreq,
                           outputDuty,
                           safeMask,
                           activeMask,
                           maskChanged,
                           inputPullupMask,
                           pullupChanged);

    case CAN_MODE_HALTECH_IO16A:
    case CAN_MODE_HALTECH_IO16B:
      return haltechIo16HandleRx(mode,
                                 msg,
                                 defaultPwmFreqHz,
                                 outputFreq,
                                 outputDuty,
                                 safeMask,
                                 activeMask,
                                 maskChanged,
                                 pullupChanged);

    case CAN_MODE_HALTECH_IO12A:
    case CAN_MODE_HALTECH_IO12B:
    case CAN_MODE_HALTECH_IO12AB:
      (void)rxBaseId;
      (void)defaultPwmFreqHz;
      return haltechIo12HandleRx(mode,
                                 msg,
                                 outputFreq,
                                 outputDuty,
                                 safeMask,
                                 activeMask,
                                 maskChanged,
                                 pullupChanged);

    case CAN_MODE_ECUMASTER_CANSWB_V3:
    case CAN_MODE_MOTEC_E888:
    case CAN_MODE_EMTRON:
    case CAN_MODE_RESERVED_9:
    case CAN_MODE_RESERVED_10:
    case CAN_MODE_RESERVED_11:
    case CAN_MODE_RESERVED_12:
    case CAN_MODE_RESERVED_13:
    case CAN_MODE_RESERVED_14:
    case CAN_MODE_RESERVED_15:
      return modeStubHandleRx(msg,
                              rxBaseId,
                              defaultPwmFreqHz,
                              outputFreq,
                              outputDuty,
                              safeMask,
                              activeMask,
                              maskChanged,
                              inputPullupMask,
                              pullupChanged);

    default:
      maskChanged = false;
      pullupChanged = false;
      return false;
  }
}

void canModeBuildTxAnalogFrames(uint8_t mode,
                                uint16_t txBaseId,
                                const uint16_t analogRaw14[8],
                                ModeTxFrame &frame0,
                                ModeTxFrame &frame1) {
  (void)txBaseId;

  if (isHaltechIo12Mode(mode)) {
    haltechIo12BuildTxAnalogFrame(mode, analogRaw14, frame0);
    frame1.id = 0;
    frame1.len = 0;
    memset(frame1.data, 0, sizeof(frame1.data));
    return;
  }

  if (isHaltechIo16Mode(mode)) {
    haltechIo16BuildTxAnalogFrames(mode, analogRaw14, frame0, frame1);
    return;
  }

  if (mode == CAN_MODE_PT_DEFAULT1) {
    mode0BuildTxAnalogFrames(txBaseId, analogRaw14, frame0, frame1);
    return;
  }
  modeStubBuildTxAnalogFrames(txBaseId, analogRaw14, frame0, frame1);
}

void canModeBuildTxStateFrame(uint8_t mode,
                              uint16_t txBaseId,
                              uint8_t digitalInMask,
                              uint8_t analogStateMask,
                              uint8_t digitalOutMask,
                              uint8_t safeMask,
                              uint8_t activeMask,
                              uint8_t fwVersion,
                              ModeTxFrame &frame) {
  (void)digitalInMask;
  (void)analogStateMask;
  (void)digitalOutMask;
  (void)safeMask;
  (void)activeMask;
  (void)fwVersion;

  if (isHaltechIo12Mode(mode)) {
    io12TxState.digitalInMask = digitalInMask;
    frame.id = 0;
    frame.len = 0;
    memset(frame.data, 0, sizeof(frame.data));
    return;
  }

  if (isHaltechIo16Mode(mode)) {
    frame.id = HALTECH_SPI_STATUS_ID_2;
    frame.len = 8;
    memset(frame.data, 0, sizeof(frame.data));
    return;
  }

  if (mode == CAN_MODE_PT_DEFAULT1) {
    mode0BuildTxStateFrame(txBaseId,
                           digitalInMask,
                           analogStateMask,
                           digitalOutMask,
                           safeMask,
                           activeMask,
                           fwVersion,
                           frame);
    return;
  }
  modeStubBuildTxStateFrame(txBaseId,
                            digitalInMask,
                            analogStateMask,
                            digitalOutMask,
                            safeMask,
                            activeMask,
                            fwVersion,
                            frame);
}

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
                               ModeTxFrame &frame) {
  (void)baseId;
  (void)timerFreq0;
  (void)period0;
  (void)high0;
  (void)hasPeriod0;
  (void)timerFreq1;
  (void)period1;
  (void)high1;
  (void)hasPeriod1;

  if (isHaltechIo12Mode(mode)) {
    uint8_t bankStart = io12BankStartForModeBank(mode, io12TxState.selectedBank);
    uint32_t shift = io12IdShiftForModeBank(mode, io12TxState.selectedBank);

    if (bankStart == 0U) {
      if (io12TxState.diCallIndex == 0U) {
        io12BuildDpiFrame1(IO12_DPI_TX_ID_1_A + shift,
                           bankStart,
                           io12TxState.digitalInMask,
                           timerFreq0,
                           period0,
                           high0,
                           hasPeriod0,
                           frame);
      } else if (io12TxState.diCallIndex == 1U) {
        io12BuildDpiFrame2(IO12_DPI_TX_ID_2_A + shift,
                           bankStart,
                           io12TxState.digitalInMask,
                           frame);
      } else {
        frame.id = 0;
        frame.len = 0;
        memset(frame.data, 0, sizeof(frame.data));
      }
    } else {
      if (io12TxState.diCallIndex == 2U) {
        io12BuildDpiFrame1(IO12_DPI_TX_ID_1_A + shift,
                           bankStart,
                           io12TxState.digitalInMask,
                           timerFreq0,
                           period0,
                           high0,
                           hasPeriod0,
                           frame);
      } else if (io12TxState.diCallIndex == 3U) {
        io12BuildDpiFrame2(IO12_DPI_TX_ID_2_A + shift,
                           bankStart,
                           io12TxState.digitalInMask,
                           frame);
      } else {
        frame.id = 0;
        frame.len = 0;
        memset(frame.data, 0, sizeof(frame.data));
      }
    }

    io12TxState.diCallIndex = static_cast<uint8_t>(io12TxState.diCallIndex + 1U);
    return;
  }

  if (isHaltechIo16Mode(mode)) {
    frame.id = 0;
    frame.len = 0;
    memset(frame.data, 0, sizeof(frame.data));
    return;
  }

  if (mode == CAN_MODE_PT_DEFAULT1) {
    mode0BuildTxDiPairFrame(baseId,
                            timerFreq0,
                            period0,
                            high0,
                            hasPeriod0,
                            timerFreq1,
                            period1,
                            high1,
                            hasPeriod1,
                            frame);
    return;
  }
  modeStubBuildTxDiPairFrame(baseId,
                             timerFreq0,
                             period0,
                             high0,
                             hasPeriod0,
                             timerFreq1,
                             period1,
                             high1,
                             hasPeriod1,
                             frame);
}

void canModeBuildTxStatusFrame(uint8_t mode,
                               uint16_t txBaseId,
                               const Mode0Status &status,
                               ModeTxFrame &frame) {
  (void)txBaseId;
  (void)status;

  if (isHaltechIo12Mode(mode)) {
    frame.id = 0;
    frame.len = 0;
    memset(frame.data, 0, sizeof(frame.data));
    return;
  }

  if (isHaltechIo16Mode(mode)) {
    frame.id = 0;
    frame.len = 0;
    memset(frame.data, 0, sizeof(frame.data));
    return;
  }

  if (mode == CAN_MODE_PT_DEFAULT1) {
    mode0BuildTxStatusFrame(txBaseId, status, frame);
    return;
  }
  modeStubBuildTxStatusFrame(txBaseId, status, frame);
}