#include "gear_outputs.h"
#include <EEPROM.h>
#include <stddef.h>

namespace {

constexpr uint16_t GEAR_OUTPUT_MAGIC = 0x474F;
constexpr uint8_t GEAR_OUTPUT_VERSION = 1;

struct GearOutputConfig {
  uint16_t magic;
  uint8_t version;
  uint32_t bindings[8];
  uint16_t checksum;
};

GearOutputConfig gearConfig = {};
bool gearReceived = false;
uint8_t currentGearByte = 0;
uint32_t gearUpdatedMs = 0;

uint16_t gearConfigChecksum() {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&gearConfig);
  uint16_t checksum = 0;
  for (size_t index = 0; index < offsetof(GearOutputConfig, checksum); index++) {
    checksum = static_cast<uint16_t>(checksum + bytes[index]);
  }
  return checksum;
}

}

void gearOutputsSave() {
  gearConfig.checksum = gearConfigChecksum();
  EEPROM.put(GEAR_OUTPUT_EEPROM_ADDRESS, gearConfig);
}

void gearOutputsResetBindings() {
  gearConfig = {};
  gearConfig.magic = GEAR_OUTPUT_MAGIC;
  gearConfig.version = GEAR_OUTPUT_VERSION;
  for (uint8_t outputIndex = 0; outputIndex < 8; outputIndex++) {
    gearConfig.bindings[outputIndex] = defaultGearBinding(outputIndex);
  }
  gearOutputsSave();
}

void gearOutputsLoad() {
  EEPROM.get(GEAR_OUTPUT_EEPROM_ADDRESS, gearConfig);
  bool valid = gearConfig.magic == GEAR_OUTPUT_MAGIC &&
               gearConfig.version == GEAR_OUTPUT_VERSION &&
               gearConfig.checksum == gearConfigChecksum();
  for (uint8_t outputIndex = 0; outputIndex < 8; outputIndex++) {
    valid = valid && ((gearConfig.bindings[outputIndex] & ~0x3FFFFUL) == 0);
  }
  if (!valid) gearOutputsResetBindings();
  gearOutputsResetSignal();
}

void gearOutputsResetSignal() {
  gearReceived = false;
  gearUpdatedMs = 0;
}

bool gearOutputsSetBinding(uint8_t outputIndex, uint32_t binding) {
  if (outputIndex >= 8 || (binding & ~0x3FFFFUL) != 0) return false;
  gearConfig.bindings[outputIndex] = binding;
  return true;
}

String gearOutputsBindingText(uint8_t outputIndex) {
  String text;
  if (outputIndex >= 8) return text;
  const char *gears = "123456789DNRPOLMSU";
  for (const char *gear = gears; *gear != '\0'; gear++) {
    uint32_t bit = gearBitForChar(*gear);
    if (bit != 0 && (gearConfig.bindings[outputIndex] & bit) != 0) {
      if (text.length() > 0) text += '|';
      text += *gear;
    }
  }
  return text.length() > 0 ? text : String("NONE");
}

bool gearOutputsHandleRx(const CanMsg &msg, const uint16_t configOutFreqHz[4],
                         uint16_t outputFreq[8], uint8_t outputDuty[8]) {
  if (msg.id != 0x470 || msg.data_length != 8) return false;
  currentGearByte = msg.data[7];
  gearReceived = true;
  gearUpdatedMs = millis();
  for (uint8_t outputIndex = 0; outputIndex < 8; outputIndex++) {
    outputDuty[outputIndex] = gearBindingDuty(gearConfig.bindings[outputIndex], currentGearByte);
    outputFreq[outputIndex] = configOutFreqHz[outputIndex / 2U];
  }
  return true;
}

bool gearOutputsSignalFresh(uint32_t nowMs, uint16_t timeoutMs) {
  return gearReceived && (nowMs - gearUpdatedMs <= timeoutMs);
}

char gearOutputsCurrentGear() {
  return gearReceived ? gearCharForByte(currentGearByte) : '?';
}