#include "gear_outputs.h"
#include "can_modes.h"
#include <EEPROM.h>
#include <cassert>
#include <iostream>

TestEeprom EEPROM;
uint32_t testNowMs = 0;

uint32_t millis() {
  return testNowMs;
}

int main() {
  std::memset(EEPROM.bytes, 0xFF, sizeof(EEPROM.bytes));
  gearOutputsLoad();
  const char *expectedBindings[] = {"1", "2", "3", "4", "5|D", "N", "R", "P"};
  for (uint8_t outputIndex = 0; outputIndex < 8; outputIndex++) {
    assert(gearOutputsBindingText(outputIndex) == expectedBindings[outputIndex]);
  }
  for (int address = 0; address < GEAR_OUTPUT_EEPROM_ADDRESS; address++) {
    assert(EEPROM.bytes[address] == 0xFF);
  }
  assert(!gearOutputsSignalFresh(0, 100));

  uint32_t binding = 0;
  assert(parseGearBinding("5|D", binding));
  assert(binding == defaultGearBinding(4));
  assert(parseGearBinding("NONE", binding) && binding == 0);
  const char *invalidBindings[] = {"", "0", "10", "X", "1|", "|1", "1||2", "NONS", "5 D"};
  for (const char *invalid : invalidBindings) {
    binding = 123;
    assert(!parseGearBinding(invalid, binding));
    assert(binding == 123);
  }
  assert(!parseGearBinding(nullptr, binding));
  assert(!gearOutputsSetBinding(8, 1));
  assert(!gearOutputsSetBinding(0, 1UL << 18));

  uint16_t configuredFreq[4] = {100, 200, 300, 400};
  uint16_t outputFreq[8] = {};
  uint8_t outputDuty[8] = {};
  uint8_t safeMask = 0xA5;
  uint8_t activeMask = 0x5A;
  uint8_t pullupMask = 0xAA;
  bool maskChanged = true;
  bool pullupChanged = true;
  CanMsg message;
  message.id = 0x470;
  message.data_length = 8;
  testNowMs = 100;

  for (uint16_t gearByte = 0; gearByte <= 255; gearByte++) {
    message.data[7] = static_cast<uint8_t>(gearByte);
    assert(canModeHandleRx(CAN_MODE_GEAR_OUTPUTS, message, 0x123, 500, configuredFreq,
                          outputFreq, outputDuty, safeMask, activeMask, maskChanged,
                          pullupMask, pullupChanged));
    for (uint8_t outputIndex = 0; outputIndex < 8; outputIndex++) {
      bool shouldLight = (outputIndex < 4 && gearByte == outputIndex + 1U) ||
                         (outputIndex == 4 && (gearByte == 5 || gearByte == 252)) ||
                         (outputIndex == 5 && gearByte == 0) ||
                         (outputIndex == 6 && gearByte == 255) ||
                         (outputIndex == 7 && gearByte == 254);
      assert(outputDuty[outputIndex] == (shouldLight ? 255 : 0));
      assert(outputFreq[outputIndex] == configuredFreq[outputIndex / 2]);
    }
    assert(safeMask == 0xA5 && activeMask == 0x5A && pullupMask == 0xAA);
    assert(!maskChanged && !pullupChanged);
  }
  assert(gearOutputsSignalFresh(200, 100));
  assert(!gearOutputsSignalFresh(201, 100));
  testNowMs = 201;
  message.id = 0x360;
  assert(!gearOutputsHandleRx(message, configuredFreq, outputFreq, outputDuty));
  message.id = 0x470;
  message.data_length = 7;
  assert(!gearOutputsHandleRx(message, configuredFreq, outputFreq, outputDuty));
  assert(!gearOutputsSignalFresh(201, 100));

  assert(parseGearBinding("6|D", binding));
  assert(gearOutputsSetBinding(0, binding));
  assert(gearOutputsSetBinding(1, binding));
  assert(gearOutputsSetBinding(4, 0));
  gearOutputsSave();
  assert(gearOutputsSetBinding(0, 0));
  gearOutputsLoad();
  assert(gearOutputsBindingText(0) == "6|D");
  assert(!gearOutputsSignalFresh(testNowMs, 100));
  message.data_length = 8;
  message.data[7] = 252;
  assert(gearOutputsHandleRx(message, configuredFreq, outputFreq, outputDuty));
  assert(outputDuty[0] == 255 && outputDuty[1] == 255 && outputDuty[4] == 0);

  EEPROM.bytes[GEAR_OUTPUT_EEPROM_ADDRESS] ^= 1;
  gearOutputsLoad();
  assert(gearOutputsBindingText(0) == "1");
  assert(gearOutputsBindingText(4) == "5|D");
  testNowMs = 0xFFFFFFF0UL;
  assert(gearOutputsHandleRx(message, configuredFreq, outputFreq, outputDuty));
  assert(gearOutputsSignalFresh(0x10, 32));
  assert(!gearOutputsSignalFresh(0x11, 32));
  gearOutputsResetSignal();
  assert(!gearOutputsSignalFresh(0x11, 100));
  assert(gearOutputsCurrentGear() == '?');

  std::cout << "Gear output tests passed: 256 gear values, CAN dispatch, bindings, persistence, timeout and wraparound.\n";
}