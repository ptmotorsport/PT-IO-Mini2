#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

// Protocol version
constexpr uint8_t PROTOCOL_VERSION = 1;

// JSON document sizes (bytes)
constexpr size_t JSON_CMD_DOC_SIZE = 512;
constexpr size_t JSON_TELEMETRY_DOC_SIZE = 1024;
constexpr size_t JSON_RESPONSE_DOC_SIZE = 256;
constexpr size_t JSON_HELLO_DOC_SIZE = 384;

// Telemetry format
enum TelemetryFormat : uint8_t {
  TELEMETRY_FORMAT_TEXT = 0,
  TELEMETRY_FORMAT_JSON = 1
};

// Telemetry subscription state
struct TelemetrySubscription {
  bool enabled;
  TelemetryFormat format;
  uint32_t intervalMs;
  uint32_t lastSendMs;
};

// JSON command handler result
struct JsonCmdResult {
  bool success;
  String errorMsg;
};

// Device state snapshot for telemetry
struct DeviceState {
  // Analog inputs
  const uint16_t* analogRaw14;
  uint8_t numAnalog;
  
  // Digital inputs
  const bool* digitalInStates;
  const uint32_t* diTimerFreq;
  volatile const uint32_t* diPeriodCounts;
  volatile const uint32_t* diHighCounts;
  volatile const bool* diHasPeriod;
  volatile const bool* diHasHigh;
  uint8_t numDigitalIn;
  
  // Digital outputs
  const uint8_t* outputDuty;
  const uint16_t* outputFreq;
  uint8_t safeMask;
  uint8_t activeMask;
  uint8_t numDigitalOut;
  
  // CAN status
  bool canInitOk;
  bool outputsInSafeState;
  uint16_t canRxCount;
  uint16_t canTxCount;
  uint16_t canTxFail;
  uint8_t canMode;
  uint32_t lastCanRxMs;
  
  // Config
  uint16_t canSpeedKbps;
  uint16_t txBaseId;
  uint16_t rxBaseId;
  uint16_t txRateHz;
  uint16_t rxTimeoutMs;
  uint8_t inputPullupMask;
  uint8_t diDebounceMs;
  uint8_t fwVersion;
  
  // Diagnostics
  uint32_t adcScanFailCount;
  uint32_t nowMs;
};

// Send JSON hello message (capabilities discovery)
void sendJsonHello(uint8_t fwVersion, uint8_t canMode);

// Send JSON response (command acknowledgment)
void sendJsonResponse(bool success, const char* msg = nullptr);

// Send JSON error
void sendJsonError(const char* msg);

// Send JSON telemetry frame
void sendJsonTelemetry(const DeviceState& state);

// Handle JSON command from GUI
// Returns result indicating success/failure
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
                                 uint8_t& serialOverrideMask);

// Get config as JSON (for getConfig command)
void sendJsonConfig(const DeviceState& state);
