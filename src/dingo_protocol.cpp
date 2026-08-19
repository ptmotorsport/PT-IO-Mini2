#include "dingo_protocol.h"
#include "neo_modes.h"
#include <string.h>

// ---------------------------------------------------------------------------
// External state from main.cpp (same "extern struct" idiom already used by
// protocol.cpp to share the Config layout without a shared header).
// ---------------------------------------------------------------------------
extern struct Config {
  uint16_t magic;
  uint8_t version;
  uint16_t canSpeedKbps;
  uint16_t txBaseId;
  uint16_t rxBaseId;
  uint16_t txRateHz;
  uint16_t rxTimeoutMs;
  uint16_t outFreqHz[4];
  uint8_t safeMask;
  uint8_t activeMask;
  uint8_t canMode;
  uint8_t inputPullupMask;
  uint8_t reserved[2];
  uint16_t crc;
} config;

extern void saveConfig();
extern void applyInputPullups();
extern uint8_t getDiDebounceMs();
extern void setDiDebounceMs(uint8_t ms);
extern uint8_t getNeoAuxMode();
extern void setNeoAuxMode(uint8_t mode);
extern uint8_t getDiActiveLowMask();
extern uint8_t getFwVersion();

extern bool diDebouncedState[8];
extern uint32_t diTimerFreq[8];
extern volatile uint32_t diPeriodCounts[8];
extern volatile uint32_t diHighCounts[8];
extern volatile bool diHasPeriod[8];
extern uint16_t adcBuf[2][8];
extern volatile uint8_t adcActiveBuf;
extern uint8_t outputDuty[8];
extern uint16_t outputFreq[8];

// ADC scaling constants — must match main.cpp's ADC_RESOLUTION(14)/AREF(5.0V).
static constexpr uint32_t DINGO_ADC_MAX_MV = 5000UL;
static constexpr uint32_t DINGO_ADC_MAX_COUNTS = 16383UL;
static constexpr uint8_t DINGO_NUM_CH = 8U;

// dingoFW CAN protocol offsets (core/device_config.h upstream).
static constexpr uint16_t DINGO_CONFIG_TX_OFFSET = 0;
static constexpr uint16_t DINGO_CONFIG_RX_OFFSET = 1;
static constexpr uint16_t DINGO_CYCLIC_TX_OFFSET = 2;

enum class DingoMsgCmd : uint8_t {
  Read = 1,
  Write = 2,
  ReadParamNotFound = 5,
  ReadAll = 10,
  ReadAllRsp = 11,
  ReadAllComplete = 12,
  ReadAllModified = 13,
  WriteAll = 20,
  WriteAllVal = 21,
  WriteAllComplete = 22,
  WriteAllModified = 23,
  BurnSettings = 30,
  Version = 31,
};

// ---------------------------------------------------------------------------
// Frame codec
// ---------------------------------------------------------------------------
struct DingoParamMsg {
  DingoMsgCmd cmd;
  uint16_t index;
  uint8_t subIndex;
  uint32_t value;
};

static void decodeParamCmd(const CanMsg &msg, DingoParamMsg &out) {
  out.cmd = static_cast<DingoMsgCmd>(msg.data[0]);
  out.index = static_cast<uint16_t>(msg.data[1] | (msg.data[2] << 8));
  out.subIndex = msg.data[3];
  out.value = static_cast<uint32_t>(msg.data[4]) |
              (static_cast<uint32_t>(msg.data[5]) << 8) |
              (static_cast<uint32_t>(msg.data[6]) << 16) |
              (static_cast<uint32_t>(msg.data[7]) << 24);
}

static void dingoSendFrame(uint32_t id, const uint8_t data[8]) {
  CanMsg tx;
  memset(&tx, 0, sizeof(tx));
  tx.id = id;
  tx.data_length = 8;
  memcpy(tx.data, data, 8);

  // Short bounded retry — response/telemetry frames are not on the
  // safety-critical hot path, but must never block the CAN drain loop.
  const uint32_t txTimeoutUs = 1000UL;
  uint32_t startUs = micros();
  bool ok = false;
  do {
    ok = CAN.write(tx);
    if (!ok) {
      delayMicroseconds(20);
    }
  } while (!ok && (micros() - startUs < txTimeoutUs));
}

static void sendParamRsp(uint16_t baseId, DingoMsgCmd cmd, uint16_t index, uint8_t subIndex, uint32_t value) {
  uint8_t data[8];
  data[0] = static_cast<uint8_t>(cmd);
  data[1] = static_cast<uint8_t>(index & 0xFF);
  data[2] = static_cast<uint8_t>((index >> 8) & 0xFF);
  data[3] = subIndex;
  data[4] = static_cast<uint8_t>(value & 0xFF);
  data[5] = static_cast<uint8_t>((value >> 8) & 0xFF);
  data[6] = static_cast<uint8_t>((value >> 16) & 0xFF);
  data[7] = static_cast<uint8_t>((value >> 24) & 0xFF);
  dingoSendFrame(baseId + DINGO_CONFIG_TX_OFFSET, data);
}

// ---------------------------------------------------------------------------
// Digital input helpers (mirror main.cpp's diActiveCounts() active-low logic)
// ---------------------------------------------------------------------------
static uint32_t diCalcFreqHz(uint8_t ch) {
  noInterrupts();
  uint32_t period = diPeriodCounts[ch];
  bool hasPeriod = diHasPeriod[ch];
  interrupts();
  uint32_t timerFreq = diTimerFreq[ch];
  if (!hasPeriod || period == 0 || timerFreq == 0) {
    return 0;
  }
  return timerFreq / period;
}

static uint32_t diCalcDutyPercent(uint8_t ch) {
  noInterrupts();
  uint32_t period = diPeriodCounts[ch];
  uint32_t high = diHighCounts[ch];
  bool hasPeriod = diHasPeriod[ch];
  interrupts();
  if (!hasPeriod || period == 0) {
    return 0;
  }
  if (high > period) {
    high = period;
  }
  bool activeLow = ((getDiActiveLowMask() >> ch) & 0x01U) != 0U;
  uint32_t activeCounts = activeLow ? (period - high) : high;
  return (activeCounts * 100UL) / period;
}

static uint32_t analogCalcMillivolts(uint8_t ch) {
  // Read from the buffer NOT currently being written by the ADC scan (same
  // pattern used by the JSON telemetry path in main.cpp).
  uint8_t readBuf = (adcActiveBuf == 0U) ? 1U : 0U;
  uint32_t raw = adcBuf[readBuf][ch];
  return (raw * DINGO_ADC_MAX_MV) / DINGO_ADC_MAX_COUNTS;
}

// ---------------------------------------------------------------------------
// Param table — indices/subindices follow the dingoFW convention (see
// DINGOCONFIG-PLAN.md). Output/DigitalInput/AnalogInput channels are 0-based.
// ---------------------------------------------------------------------------
static uint32_t getDeviceBaseId(uint8_t) { return config.txBaseId; }
static uint32_t getDeviceCanSpeed(uint8_t) { return config.canSpeedKbps; }
static uint32_t getDeviceFwVersion(uint8_t) { return getFwVersion(); }

static uint32_t getOutDuty(uint8_t ch) {
  return (static_cast<uint32_t>(outputDuty[ch]) * 100U + 127U) / 255U;
}
static bool setOutDuty(uint8_t ch, uint32_t value) {
  if (value > 100U) {
    return false;
  }
  outputDuty[ch] = static_cast<uint8_t>((value * 255U + 50U) / 100U);
  return true;
}
static uint32_t getOutFreq(uint8_t ch) { return outputFreq[ch]; }
static bool setOutFreq(uint8_t ch, uint32_t value) {
  if (value == 0 || value > 65535U) {
    return false;
  }
  uint8_t pair = ch / 2U;
  uint16_t hz = static_cast<uint16_t>(value);
  config.outFreqHz[pair] = hz;
  outputFreq[pair * 2U] = hz;
  outputFreq[pair * 2U + 1U] = hz;
  return true;
}

static uint32_t getDiDebounce(uint8_t) { return getDiDebounceMs(); }
static bool setDiDebounce(uint8_t, uint32_t value) {
  if (value > 100U) {
    return false;
  }
  setDiDebounceMs(static_cast<uint8_t>(value));
  return true;
}
static uint32_t getDiPull(uint8_t ch) { return (config.inputPullupMask >> ch) & 0x01U; }
static bool setDiPull(uint8_t ch, uint32_t value) {
  if (value > 1U) {
    return false;
  }
  uint8_t mask = static_cast<uint8_t>(1U << ch);
  config.inputPullupMask = static_cast<uint8_t>(value ? (config.inputPullupMask | mask) : (config.inputPullupMask & ~mask));
  applyInputPullups();
  return true;
}
static uint32_t getDiState(uint8_t ch) { return diDebouncedState[ch] ? 1U : 0U; }
static uint32_t getDiFreq(uint8_t ch) { return diCalcFreqHz(ch); }
static uint32_t getDiDuty(uint8_t ch) { return diCalcDutyPercent(ch); }

static uint32_t getAiEnabled(uint8_t) { return 1U; }
static uint32_t getAiMillivolts(uint8_t ch) { return analogCalcMillivolts(ch); }

static uint32_t getNeoAux(uint8_t) { return getNeoAuxMode(); }
static bool setNeoAux(uint8_t, uint32_t value) {
  if (value > 3U) {
    return false;
  }
  setNeoAuxMode(static_cast<uint8_t>(value));
  return true;
}
static uint32_t getNeoExtraPixels(uint8_t) { return neoModesGetTestExtraPixels(); }
static bool setNeoExtraPixels(uint8_t, uint32_t value) {
  if (value > 25U) {
    return false;
  }
  neoModesSetTestExtraPixels(static_cast<uint8_t>(value));
  return true;
}
static uint32_t getNeoHue(uint8_t) { return neoModesGetTestHue(); }
static bool setNeoHue(uint8_t, uint32_t value) {
  if (value > 359U) {
    return false;
  }
  neoModesSetTestHsl(static_cast<uint16_t>(value), neoModesGetTestSat(), neoModesGetTestLight());
  return true;
}
static uint32_t getNeoSat(uint8_t) { return neoModesGetTestSat(); }
static bool setNeoSat(uint8_t, uint32_t value) {
  if (value > 100U) {
    return false;
  }
  neoModesSetTestHsl(neoModesGetTestHue(), static_cast<uint8_t>(value), neoModesGetTestLight());
  return true;
}
static uint32_t getNeoLight(uint8_t) { return neoModesGetTestLight(); }
static bool setNeoLight(uint8_t, uint32_t value) {
  if (value > 100U) {
    return false;
  }
  neoModesSetTestHsl(neoModesGetTestHue(), neoModesGetTestSat(), static_cast<uint8_t>(value));
  return true;
}

struct DingoParamEntry {
  uint16_t baseIndex;
  uint8_t subIndex;
  uint8_t count;
  uint32_t (*getValue)(uint8_t ch);
  bool (*setValue)(uint8_t ch, uint32_t value);  // nullptr = read-only
};

// Index ranges follow dingoFW's convention: Device=0x0000, Outputs=0x1000+,
// DigitalInputs=0x1200+, AnalogInputs=0x2200+. NeoPixel (0x4000) has no
// dingoFW equivalent — see the TODO list in DINGOCONFIG-PLAN.md.
static const DingoParamEntry DINGO_PARAMS[] = {
  {0x0000, 0, 1, getDeviceBaseId, nullptr},
  {0x0000, 1, 1, getDeviceCanSpeed, nullptr},
  {0x0000, 2, 1, getDeviceFwVersion, nullptr},

  {0x1000, 0, DINGO_NUM_CH, getOutDuty, setOutDuty},
  {0x1000, 1, DINGO_NUM_CH, getOutFreq, setOutFreq},

  {0x1200, 0, DINGO_NUM_CH, getDiDebounce, setDiDebounce},
  {0x1200, 1, DINGO_NUM_CH, getDiPull, setDiPull},
  {0x1200, 2, DINGO_NUM_CH, getDiState, nullptr},
  {0x1200, 3, DINGO_NUM_CH, getDiFreq, nullptr},
  {0x1200, 4, DINGO_NUM_CH, getDiDuty, nullptr},

  {0x2200, 0, DINGO_NUM_CH, getAiEnabled, nullptr},
  {0x2200, 1, DINGO_NUM_CH, getAiMillivolts, nullptr},

  {0x4000, 0, 1, getNeoAux, setNeoAux},
  {0x4000, 1, 1, getNeoExtraPixels, setNeoExtraPixels},
  {0x4000, 2, 1, getNeoHue, setNeoHue},
  {0x4000, 3, 1, getNeoSat, setNeoSat},
  {0x4000, 4, 1, getNeoLight, setNeoLight},
};
static constexpr size_t DINGO_PARAM_COUNT = sizeof(DINGO_PARAMS) / sizeof(DINGO_PARAMS[0]);

static const DingoParamEntry *findParam(uint16_t index, uint8_t subIndex, uint8_t &chOut) {
  for (size_t i = 0; i < DINGO_PARAM_COUNT; i++) {
    const DingoParamEntry &entry = DINGO_PARAMS[i];
    if (index < entry.baseIndex) {
      continue;
    }
    uint16_t ch = index - entry.baseIndex;
    if (ch < entry.count && entry.subIndex == subIndex) {
      chOut = static_cast<uint8_t>(ch);
      return &entry;
    }
  }
  return nullptr;
}

// ---------------------------------------------------------------------------
// Command handling
// ---------------------------------------------------------------------------
static void handleReadAll(uint16_t baseId) {
  // Bounded, explicit-action-only burst (triggered by a user "Read Config"
  // click, never from the periodic cyclic path) — bench-verified acceptable
  // one-off latency, see DINGOCONFIG-PLAN.md Phase 2 verification notes.
  for (size_t i = 0; i < DINGO_PARAM_COUNT; i++) {
    const DingoParamEntry &entry = DINGO_PARAMS[i];
    for (uint8_t ch = 0; ch < entry.count; ch++) {
      uint16_t index = static_cast<uint16_t>(entry.baseIndex + ch);
      uint32_t value = entry.getValue(ch);
      sendParamRsp(baseId, DingoMsgCmd::ReadAllRsp, index, entry.subIndex, value);
      delayMicroseconds(50);
    }
  }
  sendParamRsp(baseId, DingoMsgCmd::ReadAllComplete, 0, 0, 0);
}

bool dingoHandleRx(const CanMsg &msg,
                   uint16_t baseId,
                   uint16_t /*outputFreq*/[8],
                   uint8_t /*outputDuty*/[8]) {
  if (msg.id != (static_cast<uint32_t>(baseId) + DINGO_CONFIG_RX_OFFSET)) {
    return false;
  }
  if (msg.data_length != 8) {
    return true;  // frame was for us, but malformed — ignore, do not crash/hang
  }

  DingoParamMsg cmdMsg;
  decodeParamCmd(msg, cmdMsg);

  switch (cmdMsg.cmd) {
    case DingoMsgCmd::Read: {
      uint8_t ch = 0;
      const DingoParamEntry *entry = findParam(cmdMsg.index, cmdMsg.subIndex, ch);
      if (entry == nullptr) {
        sendParamRsp(baseId, DingoMsgCmd::ReadParamNotFound, cmdMsg.index, cmdMsg.subIndex, 0);
        return true;
      }
      sendParamRsp(baseId, DingoMsgCmd::Read, cmdMsg.index, cmdMsg.subIndex, entry->getValue(ch));
      return true;
    }

    case DingoMsgCmd::Write:
    case DingoMsgCmd::WriteAllVal: {
      uint8_t ch = 0;
      const DingoParamEntry *entry = findParam(cmdMsg.index, cmdMsg.subIndex, ch);
      if (entry == nullptr || entry->setValue == nullptr || !entry->setValue(ch, cmdMsg.value)) {
        if (cmdMsg.cmd == DingoMsgCmd::Write) {
          sendParamRsp(baseId, DingoMsgCmd::ReadParamNotFound, cmdMsg.index, cmdMsg.subIndex, 0);
        }
        return true;
      }
      if (cmdMsg.cmd == DingoMsgCmd::Write) {
        sendParamRsp(baseId, DingoMsgCmd::Write, cmdMsg.index, cmdMsg.subIndex, cmdMsg.value);
      }
      return true;
    }

    case DingoMsgCmd::ReadAll:
    case DingoMsgCmd::ReadAllModified:
      handleReadAll(baseId);
      return true;

    case DingoMsgCmd::WriteAll:
      sendParamRsp(baseId, DingoMsgCmd::WriteAll, 0, 0, 0);
      return true;

    case DingoMsgCmd::WriteAllComplete:
    case DingoMsgCmd::WriteAllModified:
      sendParamRsp(baseId, cmdMsg.cmd, 0, 0, 0);
      return true;

    case DingoMsgCmd::Version:
      sendParamRsp(baseId, DingoMsgCmd::Version, 0, 0, getFwVersion());
      return true;

    case DingoMsgCmd::BurnSettings:
      saveConfig();
      sendParamRsp(baseId, DingoMsgCmd::BurnSettings, 0, 0, 1);
      return true;

    default:
      // Unsupported command (Sleep/Bootloader/CheckCrc) — Mini2 has no
      // CAN-sleep and uses USB dfu-util for firmware updates. Ignore silently.
      return true;
  }
}

// ---------------------------------------------------------------------------
// Cyclic telemetry frames
// ---------------------------------------------------------------------------
static uint16_t s_analogMvCache[DINGO_NUM_CH] = {0};
static uint8_t s_heartbeat = 0;

void dingoCacheAnalogRaw(const uint16_t analogRaw14[8]) {
  for (uint8_t ch = 0; ch < DINGO_NUM_CH; ch++) {
    s_analogMvCache[ch] = static_cast<uint16_t>((static_cast<uint32_t>(analogRaw14[ch]) * DINGO_ADC_MAX_MV) / DINGO_ADC_MAX_COUNTS);
  }
}

// msg0 (txBaseId + CYCLIC_TX_OFFSET + 0): digital input state bitmask + device status + heartbeat.
void dingoBuildStateFrame(uint16_t txBaseId, uint8_t digitalInMask, uint8_t fwVersion, ModeTxFrame &frame) {
  frame.id = txBaseId + DINGO_CYCLIC_TX_OFFSET + 0;
  frame.len = 8;
  memset(frame.data, 0, sizeof(frame.data));
  frame.data[0] = digitalInMask;
  frame.data[1] = 0;  // DeviceState: 0 = Run (Mini2 has no sleep/overtemp/error state to report yet)
  frame.data[2] = s_heartbeat++;
  frame.data[3] = fwVersion;
}

// msg1..msg4 (txBaseId + CYCLIC_TX_OFFSET + 1..4): repurposed DiPair slots.
// Sequenced purely from (baseId - txBaseId) so no extra call-index state is needed.
void dingoBuildDiPairFrame(uint32_t baseId, ModeTxFrame &frame) {
  uint32_t offset = baseId - config.txBaseId;
  frame.id = baseId;
  frame.len = 8;
  memset(frame.data, 0, sizeof(frame.data));

  // Pack little-endian manually (frame.data has no alignment guarantee for a
  // uint16_t reinterpret_cast) — matches the byte-packing convention already
  // used throughout this file for every other CAN mode.
  auto packU16 = [&](uint8_t byteOffset, uint16_t value) {
    frame.data[byteOffset] = static_cast<uint8_t>(value & 0xFF);
    frame.data[byteOffset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  };

  switch (offset) {
    case DINGO_CYCLIC_TX_OFFSET + 1:  // AI1-4 millivolts
      packU16(0, s_analogMvCache[0]);
      packU16(2, s_analogMvCache[1]);
      packU16(4, s_analogMvCache[2]);
      packU16(6, s_analogMvCache[3]);
      break;
    case DINGO_CYCLIC_TX_OFFSET + 2:  // AI5-8 millivolts
      packU16(0, s_analogMvCache[4]);
      packU16(2, s_analogMvCache[5]);
      packU16(4, s_analogMvCache[6]);
      packU16(6, s_analogMvCache[7]);
      break;
    case DINGO_CYCLIC_TX_OFFSET + 3:  // Output duty % (1-8)
      for (uint8_t ch = 0; ch < DINGO_NUM_CH; ch++) {
        frame.data[ch] = static_cast<uint8_t>(getOutDuty(ch));
      }
      break;
    case DINGO_CYCLIC_TX_OFFSET + 4:  // Output pair frequency (Hz), one per GPT pair
      packU16(0, outputFreq[0]);
      packU16(2, outputFreq[2]);
      packU16(4, outputFreq[4]);
      packU16(6, outputFreq[6]);
      break;
    default:
      frame.len = 0;
      break;
  }
}

// msg5 (txBaseId + CYCLIC_TX_OFFSET + 5): NeoPixel advanced-feature status.
void dingoBuildStatusFrame(uint16_t txBaseId, ModeTxFrame &frame) {
  frame.id = txBaseId + DINGO_CYCLIC_TX_OFFSET + 5;
  frame.len = 8;
  memset(frame.data, 0, sizeof(frame.data));
  frame.data[0] = getNeoAuxMode();
  frame.data[1] = neoModesGetTestExtraPixels();
  uint16_t hue = neoModesGetTestHue();
  frame.data[2] = static_cast<uint8_t>(hue & 0xFF);
  frame.data[3] = static_cast<uint8_t>((hue >> 8) & 0xFF);
  frame.data[4] = neoModesGetTestSat();
  frame.data[5] = neoModesGetTestLight();
}
