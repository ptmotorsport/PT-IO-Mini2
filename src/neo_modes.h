#pragma once

#include <Arduino.h>
#include <Arduino_CAN.h>

enum NeoAuxMode : uint8_t {
  NEO_AUX_NONE = 0,
  NEO_AUX_SHIFTLIGHT = 1,
  NEO_AUX_GEAR = 2,
  NEO_AUX_TEST = 3,
};

const char *neoAuxModeName(uint8_t mode);

void neoModesInit();
void neoModesUpdateFromCan(const CanMsg &msg, uint32_t nowMs);
void neoModesRender(uint8_t auxMode,
                    uint32_t nowMs,
                    uint32_t *frame,
                    uint8_t frameCount,
                    uint8_t auxStartIndex);

uint8_t neoModesGetTestExtraPixels();
void neoModesSetTestExtraPixels(uint8_t count);
uint8_t neoModesGetAuxBrightness();
void neoModesSetAuxBrightness(uint8_t brightness);
uint16_t neoModesGetTestHue();
uint8_t neoModesGetTestSat();
uint8_t neoModesGetTestLight();
void neoModesSetTestHsl(uint16_t hue, uint8_t sat, uint8_t light);
