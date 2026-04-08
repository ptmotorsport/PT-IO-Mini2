#include "protocol.h"
#include "can_modes.h"

// External config structure (defined in main.cpp)
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

// External functions from main.cpp
extern void saveConfig();
extern void applyOutputs(bool useSafeState);
extern void applyInputPullups();
extern bool setCanBitrate(uint16_t kbps);
extern uint8_t getDiDebounceMs();
extern void setDiDebounceMs(uint8_t ms);

// Send JSON hello message on connect
void sendJsonHello(uint8_t fwVersion, uint8_t canMode) {
  StaticJsonDocument<JSON_HELLO_DOC_SIZE> doc;
  
  doc["type"] = "hello";
  doc["version"] = PROTOCOL_VERSION;
  doc["fw"] = fwVersion;
  doc["canMode"] = canMode;
  doc["canModeName"] = canModeName(canMode);
  
  JsonArray capabilities = doc.createNestedArray("capabilities");
  capabilities.add("analog");
  capabilities.add("digital");
  capabilities.add("pwm");
  capabilities.add("can");
  capabilities.add("config");
  
  doc["analogChannels"] = 8;
  doc["digitalIn"] = 8;
  doc["digitalOut"] = 8;
  
  serializeJson(doc, Serial);
  Serial.println();
}

// Send JSON response
void sendJsonResponse(bool success, const char* msg) {
  StaticJsonDocument<JSON_RESPONSE_DOC_SIZE> doc;
  
  doc["type"] = "response";
  doc["status"] = success ? "ok" : "error";
  if (msg != nullptr) {
    doc["msg"] = msg;
  }
  
  serializeJson(doc, Serial);
  Serial.println();
}

// Send JSON error
void sendJsonError(const char* msg) {
  StaticJsonDocument<JSON_RESPONSE_DOC_SIZE> doc;
  
  doc["type"] = "error";
  doc["msg"] = msg;
  
  serializeJson(doc, Serial);
  Serial.println();
}

// Send JSON telemetry frame
void sendJsonTelemetry(const DeviceState& state) {
  StaticJsonDocument<JSON_TELEMETRY_DOC_SIZE> doc;
  
  doc["type"] = "telemetry";
  doc["timestamp"] = state.nowMs;
  
  // Analog inputs (14-bit values)
  JsonArray analog = doc.createNestedArray("analog");
  for (uint8_t i = 0; i < state.numAnalog; i++) {
    analog.add(state.analogRaw14[i]);
  }
  
  // Digital inputs (state + frequency + duty)
  JsonArray digital = doc.createNestedArray("digital");
  for (uint8_t i = 0; i < state.numDigitalIn; i++) {
    JsonObject di = digital.createNestedObject();
    di["state"] = state.digitalInStates[i];
    
    if (state.diHasPeriod[i] && state.diTimerFreq[i] > 0 && state.diPeriodCounts[i] > 0) {
      // Calculate frequency in Hz
      float freqHz = static_cast<float>(state.diTimerFreq[i]) / static_cast<float>(state.diPeriodCounts[i]);
      di["freq"] = serialized(String(freqHz, 2));  // 2 decimal places
      
      // Calculate duty cycle percentage
      if (state.diHasHigh[i]) {
        float dutyPct = (static_cast<float>(state.diHighCounts[i]) / static_cast<float>(state.diPeriodCounts[i])) * 100.0f;
        di["duty"] = serialized(String(dutyPct, 1));  // 1 decimal place
      } else {
        di["duty"] = nullptr;
      }
    } else {
      di["freq"] = nullptr;
      di["duty"] = nullptr;
    }
  }
  
  // Digital outputs (duty + frequency + state flags)
  JsonArray outputs = doc.createNestedArray("outputs");
  for (uint8_t i = 0; i < state.numDigitalOut; i++) {
    JsonObject out = outputs.createNestedObject();
    
    // Duty as percentage
    float dutyPct = (static_cast<float>(state.outputDuty[i]) / 255.0f) * 100.0f;
    out["duty"] = serialized(String(dutyPct, 1));
    
    // Frequency (Hz)
    out["freq"] = state.outputFreq[i];
    
    // Safe state flag (what this output does in safe mode)
    bool safeState = (state.safeMask & (1 << i)) != 0;
    out["safe"] = safeState;
    
    // Active polarity (LOW=normal, HIGH=inverted)
    bool activeHigh = (state.activeMask & (1 << i)) != 0;
    out["activeHigh"] = activeHigh;
  }
  
  // CAN status
  JsonObject can = doc.createNestedObject("can");
  can["init"] = state.canInitOk;
  can["safeState"] = state.outputsInSafeState;
  can["rxCount"] = state.canRxCount;
  can["txCount"] = state.canTxCount;
  can["txFail"] = state.canTxFail;
  can["mode"] = state.canMode;
  can["lastRxMs"] = state.lastCanRxMs;
  can["rxTimeout"] = state.rxTimeoutMs;
  
  serializeJson(doc, Serial);
  Serial.println();
}

// Send JSON config
void sendJsonConfig(const DeviceState& state) {
  StaticJsonDocument<JSON_RESPONSE_DOC_SIZE> doc;
  
  doc["type"] = "config";
  doc["canSpeed"] = state.canSpeedKbps;
  doc["txBaseId"] = state.txBaseId;
  doc["rxBaseId"] = state.rxBaseId;
  doc["txRate"] = state.txRateHz;
  doc["rxTimeout"] = state.rxTimeoutMs;
  doc["canMode"] = state.canMode;
  doc["safeMask"] = state.safeMask;
  doc["activeMask"] = state.activeMask;
  doc["inputPullupMask"] = state.inputPullupMask;
  doc["diDebounceMs"] = state.diDebounceMs;
  
  // Output frequencies (per pair)
  JsonArray outFreqs = doc.createNestedArray("outFreq");
  for (uint8_t i = 0; i < state.numDigitalOut; i++) {
    outFreqs.add(state.outputFreq[i]);
  }
  
  serializeJson(doc, Serial);
  Serial.println();
}

// Handle JSON commands
JsonCmdResult handleJsonCommand(const String& jsonLine,
                                 TelemetrySubscription& telemetrySub,
                                 uint8_t outputDuty[],
                                 uint16_t outputFreq[],
                                 uint8_t& safeMask,
                                 uint8_t& activeMask,
                                 uint8_t& inputPullupMask,
                                 bool& configChanged,
                                 bool& outputsChanged,
                                 bool& pullupChanged,
                                 uint8_t& serialOverrideMask) {
  
  StaticJsonDocument<JSON_CMD_DOC_SIZE> doc;
  DeserializationError error = deserializeJson(doc, jsonLine);
  
  if (error) {
    return {false, String("JSON parse error: ") + error.c_str()};
  }
  
  const char* cmd = doc["cmd"];
  if (cmd == nullptr) {
    return {false, "Missing 'cmd' field"};
  }
  
  // Subscribe to telemetry streaming
  if (strcmp(cmd, "subscribe") == 0) {
    const char* stream = doc["stream"];
    if (stream == nullptr || strcmp(stream, "telemetry") != 0) {
      return {false, "Unknown stream type (use 'telemetry')"};
    }
    
    uint32_t interval = doc["interval"] | 100;  // Default 100ms
    if (interval < 10) {
      return {false, "Interval must be >= 10ms"};
    }
    
    telemetrySub.enabled = true;
    telemetrySub.format = TELEMETRY_FORMAT_JSON;
    telemetrySub.intervalMs = interval;
    telemetrySub.lastSendMs = 0;  // Force immediate send
    
    return {true, ""};
  }
  
  // Unsubscribe from telemetry
  if (strcmp(cmd, "unsubscribe") == 0) {
    telemetrySub.enabled = false;
    return {true, ""};
  }
  
  // Get hello message (device info / capabilities)
  if (strcmp(cmd, "getHello") == 0) {
    // This will be handled by caller to send hello message
    return {true, "getHello"};  // Special marker
  }
  
  // Set output (individual channel)
  if (strcmp(cmd, "setOutput") == 0) {
    uint8_t ch = doc["ch"] | 0;
    int duty = doc["duty"] | -1;
    
    if (ch < 1 || ch > 8) {
      return {false, "Channel must be 1-8"};
    }
    if (duty < 0 || duty > 100) {
      return {false, "Duty must be 0-100"};
    }
    
    outputDuty[ch - 1] = static_cast<uint8_t>((duty * 255 + 50) / 100);
    // Set this channel's override bit (enable serial control)
    serialOverrideMask |= (1 << (ch - 1));
    outputsChanged = true;
    
    return {true, ""};
  }
  
  // Set output frequency
  if (strcmp(cmd, "setOutputFreq") == 0) {
    uint8_t ch = doc["ch"] | 0;
    uint16_t freq = doc["freq"] | 0;
    
    if (ch < 1 || ch > 8) {
      return {false, "Channel must be 1-8"};
    }
    if (freq == 0) {
      return {false, "Frequency must be > 0"};
    }
    
    // Update pair frequency
    uint8_t pair = (ch - 1) / 2;
    config.outFreqHz[pair] = freq;
    outputFreq[pair * 2] = freq;
    outputFreq[pair * 2 + 1] = freq;
    configChanged = true;
    outputsChanged = true;
    
    return {true, ""};
  }
  
  // Set safe state for a channel
  if (strcmp(cmd, "setSafe") == 0) {
    uint8_t ch = doc["ch"] | 0;
    bool val = doc["value"] | false;
    
    if (ch < 1 || ch > 8) {
      return {false, "Channel must be 1-8"};
    }
    
    uint8_t mask = 1 << (ch - 1);
    safeMask = (safeMask & ~mask) | (val ? mask : 0);
    config.safeMask = safeMask;
    configChanged = true;
    outputsChanged = true;
    
    return {true, ""};
  }
  
  // Set active polarity for a channel
  if (strcmp(cmd, "setActive") == 0) {
    uint8_t ch = doc["ch"] | 0;
    bool activeHigh = doc["activeHigh"] | false;
    
    if (ch < 1 || ch > 8) {
      return {false, "Channel must be 1-8"};
    }
    
    uint8_t mask = 1 << (ch - 1);
    activeMask = (activeMask & ~mask) | (activeHigh ? mask : 0);
    config.activeMask = activeMask;
    configChanged = true;
    outputsChanged = true;
    
    return {true, ""};
  }
  
  // Set input pullup
  if (strcmp(cmd, "setInputPullup") == 0) {
    uint8_t ch = doc["ch"] | 0;
    bool enabled = doc["enabled"] | false;
    
    if (ch < 1 || ch > 8) {
      return {false, "Channel must be 1-8"};
    }
    
    uint8_t mask = 1 << (ch - 1);
    inputPullupMask = (inputPullupMask & ~mask) | (enabled ? mask : 0);
    config.inputPullupMask = inputPullupMask;
    configChanged = true;
    pullupChanged = true;
    
    return {true, ""};
  }
  
  // Set input debounce
  if (strcmp(cmd, "setDebounce") == 0) {
    uint8_t ms = doc["ms"] | 0;
    
    if (ms > 100) {
      return {false, "Debounce must be 0-100 ms"};
    }
    
    setDiDebounceMs(ms);
    configChanged = true;
    
    return {true, ""};
  }
  
  // Get current config (sends separate JSON config message)
  if (strcmp(cmd, "getConfig") == 0) {
    // This will be handled by caller to build DeviceState and call sendJsonConfig
    return {true, "getConfig"};  // Special marker
  }
  
  // Get status (sends one telemetry frame)
  if (strcmp(cmd, "getStatus") == 0) {
    // This will be handled by caller to send telemetry
    return {true, "getStatus"};  // Special marker
  }
  
  // Set CAN configuration
  if (strcmp(cmd, "setCanConfig") == 0) {
    bool changed = false;
    
    if (doc.containsKey("canSpeed")) {
      uint16_t speed = doc["canSpeed"];
      if (speed != 125 && speed != 250 && speed != 500 && speed != 1000) {
        return {false, "CAN speed must be 125, 250, 500, or 1000"};
      }
      if (setCanBitrate(speed)) {
        config.canSpeedKbps = speed;
        changed = true;
      }
    }
    
    if (doc.containsKey("txBaseId")) {
      uint16_t id = doc["txBaseId"];
      if (id > 0x7FF) {
        return {false, "CAN ID must be 0x000-0x7FF"};
      }
      config.txBaseId = id;
      changed = true;
    }
    
    if (doc.containsKey("rxBaseId")) {
      uint16_t id = doc["rxBaseId"];
      if (id > 0x7FF) {
        return {false, "CAN ID must be 0x000-0x7FF"};
      }
      config.rxBaseId = id;
      changed = true;
    }
    
    if (doc.containsKey("txRate")) {
      uint16_t rate = doc["txRate"];
      if (rate == 0) {
        return {false, "TX rate must be > 0"};
      }
      config.txRateHz = rate;
      changed = true;
    }
    
    if (doc.containsKey("rxTimeout")) {
      uint16_t timeout = doc["rxTimeout"];
      if (timeout < 100) {
        return {false, "RX timeout must be >= 100ms"};
      }
      config.rxTimeoutMs = timeout;
      changed = true;
    }
    
    if (doc.containsKey("canMode")) {
      uint8_t mode = doc["canMode"];
      if (mode >= 16) {  // CAN_MODE_COUNT
        return {false, "CAN mode must be 0-15"};
      }
      config.canMode = mode;
      changed = true;
    }
    
    if (changed) {
      configChanged = true;
    }
    
    return {true, ""};
  }
  
  // Reset to defaults
  if (strcmp(cmd, "resetDefaults") == 0) {
    // This will be handled by caller (needs access to setDefaults function)
    return {true, "resetDefaults"};  // Special marker
  }
  
  // Set serial override (per-channel or all)
  if (strcmp(cmd, "setSerialOverride") == 0) {
    if (doc.containsKey("mask")) {
      uint8_t mask = doc["mask"] | 0;
      serialOverrideMask = mask;
    } else if (doc.containsKey("ch")) {
      uint8_t ch = doc["ch"] | 0;
      bool enabled = doc["enabled"] | false;
      
      if (ch < 1 || ch > 8) {
        return {false, "Channel must be 1-8"};
      }
      
      uint8_t bit = 1 << (ch - 1);
      if (enabled) {
        serialOverrideMask |= bit;
      } else {
        serialOverrideMask &= ~bit;
      }
    } else {
      return {false, "Requires 'mask' or 'ch'+'enabled' fields"};
    }
    return {true, ""};
  }
  
  // Unknown command
  return {false, String("Unknown command: ") + cmd};
}
