#pragma once

#include <Arduino.h>
#include <Arduino_CAN.h>
#include "gear_logic.h"

constexpr int GEAR_OUTPUT_EEPROM_ADDRESS = 128;

void gearOutputsLoad();
void gearOutputsSave();
void gearOutputsResetBindings();
void gearOutputsResetSignal();
bool gearOutputsSetBinding(uint8_t outputIndex, uint32_t binding);
String gearOutputsBindingText(uint8_t outputIndex);
bool gearOutputsHandleRx(const CanMsg &msg, const uint16_t configOutFreqHz[4],
                         uint16_t outputFreq[8], uint8_t outputDuty[8]);
bool gearOutputsSignalFresh(uint32_t nowMs, uint16_t timeoutMs);
char gearOutputsCurrentGear();