#include "neo_modes.h"

namespace {

constexpr uint8_t NEO_SHIFTLIGHT_COUNT = 8;
constexpr uint8_t NEO_GEAR_MATRIX_SIZE = 25;

uint16_t neoRpm = 0;
uint32_t neoRpmUpdatedMs = 0;
char neoGearChar = 'N';
uint32_t neoGearUpdatedMs = 0;

uint8_t neoTestExtraPixels = NEO_SHIFTLIGHT_COUNT;
uint16_t neoTestHue = 0;
uint8_t neoTestSat = 100;
uint8_t neoTestLight = 40;

static uint32_t neoColor(uint8_t r, uint8_t g, uint8_t b) {
  return (static_cast<uint32_t>(r) << 16)
       | (static_cast<uint32_t>(g) << 8)
       | static_cast<uint32_t>(b);
}

static char decodeHaltechGearByte(uint8_t gearByte) {
  switch (gearByte) {
    case 0: return 'N';
    case 1: return '1';
    case 2: return '2';
    case 3: return '3';
    case 4: return '4';
    case 5: return '5';
    case 6: return '6';
    case 7: return '7';
    case 8: return '8';
    case 9: return '9';
    case 248: return 'O';
    case 249: return 'L';
    case 250: return 'M';
    case 251: return 'S';
    case 252: return 'D';
    case 253: return 'U';
    case 254: return 'P';
    case 255: return 'R';
    default: return 'U';
  }
}

static size_t neoMatrixIndex(uint8_t row, uint8_t col, uint8_t auxStartIndex) {
  if (row > 4 || col > 4) {
    return auxStartIndex;
  }
  uint8_t colOffset = (row % 2 == 0) ? static_cast<uint8_t>(4 - col) : col;
  return static_cast<size_t>(auxStartIndex + row * 5 + colOffset);
}

static uint32_t hslToRgb(uint16_t hueDeg, uint8_t satPct, uint8_t lightPct) {
  float h = static_cast<float>(hueDeg % 360U) / 360.0f;
  float s = static_cast<float>(satPct) / 100.0f;
  float l = static_cast<float>(lightPct) / 100.0f;

  float r, g, b;
  if (s <= 0.0001f) {
    r = l;
    g = l;
    b = l;
  } else {
    auto hue2rgb = [](float p, float q, float t) -> float {
      if (t < 0.0f) t += 1.0f;
      if (t > 1.0f) t -= 1.0f;
      if (t < 1.0f / 6.0f) return p + (q - p) * 6.0f * t;
      if (t < 1.0f / 2.0f) return q;
      if (t < 2.0f / 3.0f) return p + (q - p) * (2.0f / 3.0f - t) * 6.0f;
      return p;
    };

    float q = (l < 0.5f) ? (l * (1.0f + s)) : (l + s - l * s);
    float p = 2.0f * l - q;
    r = hue2rgb(p, q, h + 1.0f / 3.0f);
    g = hue2rgb(p, q, h);
    b = hue2rgb(p, q, h - 1.0f / 3.0f);
  }

  uint8_t rr = static_cast<uint8_t>(r * 255.0f + 0.5f);
  uint8_t gg = static_cast<uint8_t>(g * 255.0f + 0.5f);
  uint8_t bb = static_cast<uint8_t>(b * 255.0f + 0.5f);
  return neoColor(rr, gg, bb);
}

static void neoRenderShiftlight(uint32_t nowMs,
                                uint32_t *frame,
                                uint8_t frameCount,
                                uint8_t auxStartIndex) {
  if (frame == nullptr || frameCount <= auxStartIndex) {
    return;
  }

  const uint8_t auxCount = static_cast<uint8_t>(frameCount - auxStartIndex);
  const uint8_t useCount = auxCount < NEO_SHIFTLIGHT_COUNT ? auxCount : NEO_SHIFTLIGHT_COUNT;
  if (useCount == 0 || (nowMs - neoRpmUpdatedMs) > 1000UL || neoRpm == 0U) {
    return;
  }

  constexpr uint16_t STARTRPM = 4500;
  constexpr uint16_t ENDRPM = 6800;
  constexpr uint16_t FLASHRPM = 7200;

  const uint32_t ledColors[NEO_SHIFTLIGHT_COUNT] = {
    neoColor(0, 150, 0),
    neoColor(0, 150, 0),
    neoColor(0, 150, 0),
    neoColor(0, 150, 0),
    neoColor(150, 80, 0),
    neoColor(150, 80, 0),
    neoColor(150, 0, 0),
    neoColor(150, 0, 0),
  };

  if (neoRpm >= FLASHRPM) {
    bool flashOn = ((nowMs / 60U) % 2U) == 0U;
    uint32_t flashColor = flashOn ? neoColor(150, 150, 150) : neoColor(0, 0, 0);
    for (uint8_t i = 0; i < useCount; i++) {
      frame[auxStartIndex + i] = flashColor;
    }
    return;
  }

  if (neoRpm >= ENDRPM) {
    for (uint8_t i = 0; i < useCount; i++) {
      frame[auxStartIndex + i] = neoColor(120, 0, 0);
    }
    return;
  }

  uint16_t step = static_cast<uint16_t>((ENDRPM - STARTRPM) / NEO_SHIFTLIGHT_COUNT);
  for (uint8_t i = 0; i < useCount; i++) {
    uint16_t threshold = static_cast<uint16_t>(STARTRPM + (step * i));
    if (neoRpm > threshold) {
      frame[auxStartIndex + i] = ledColors[i];
    }
  }
}

static void neoRenderGear(uint32_t nowMs,
                          uint32_t *frame,
                          uint8_t frameCount,
                          uint8_t auxStartIndex) {
  if (frame == nullptr || frameCount < (auxStartIndex + NEO_GEAR_MATRIX_SIZE)) {
    return;
  }
  if ((nowMs - neoGearUpdatedMs) > 2000UL) {
    return;
  }

  const bool *pattern = nullptr;
  uint32_t color = neoColor(150, 0, 120);

  static const bool CHAR_R[15] = {1,1,1,0,0, 0,0,1,0,1, 1,1,1,0,1};
  static const bool CHAR_N[15] = {1,1,1,0,0, 0,0,1,0,0, 1,1,1,0,0};
  static const bool CHAR_1[15] = {0,0,0,0,0, 0,0,0,0,0, 1,1,1,1,1};
  static const bool CHAR_2[15] = {1,1,1,0,1, 1,0,1,0,1, 1,0,1,1,1};
  static const bool CHAR_3[15] = {1,0,1,0,1, 1,0,1,0,1, 1,1,1,1,1};
  static const bool CHAR_4[15] = {0,0,1,1,1, 0,0,1,0,0, 1,1,1,1,1};
  static const bool CHAR_5[15] = {1,0,1,1,1, 1,0,1,0,1, 1,1,1,0,1};
  static const bool CHAR_6[15] = {1,1,1,1,1, 1,0,1,0,1, 1,1,1,0,1};
  static const bool CHAR_7[15] = {0,0,0,0,1, 0,0,0,0,1, 1,1,1,1,1};
  static const bool CHAR_8[15] = {1,1,1,1,1, 1,0,1,0,1, 1,1,1,1,1};
  static const bool CHAR_9[15] = {1,0,0,1,1, 1,0,1,0,1, 1,1,1,1,1};
  static const bool CHAR_O[15] = {1,1,1,1,1, 1,0,0,0,1, 1,1,1,1,1};
  static const bool CHAR_L[15] = {1,1,1,1,1, 1,0,0,0,0, 1,0,0,0,0};
  static const bool CHAR_M[15] = {1,1,0,1,1, 1,0,1,0,1, 1,0,1,0,1};
  static const bool CHAR_S[15] = {1,0,1,1,1, 1,0,1,0,1, 1,1,1,0,1};
  static const bool CHAR_D[15] = {1,1,1,0,0, 1,0,1,0,0, 1,1,1,1,1};
  static const bool CHAR_U[15] = {1,1,1,1,1, 1,0,0,0,0, 1,1,1,1,1};
  static const bool CHAR_P[15] = {1,1,1,1,1, 0,0,1,0,1, 0,0,1,1,1};

  switch (neoGearChar) {
    case 'R': pattern = CHAR_R; color = neoColor(150, 0, 0); break;
    case 'N': pattern = CHAR_N; color = neoColor(150, 100, 0); break;
    case '1': pattern = CHAR_1; break;
    case '2': pattern = CHAR_2; break;
    case '3': pattern = CHAR_3; break;
    case '4': pattern = CHAR_4; break;
    case '5': pattern = CHAR_5; break;
    case '6': pattern = CHAR_6; break;
    case '7': pattern = CHAR_7; break;
    case '8': pattern = CHAR_8; break;
    case '9': pattern = CHAR_9; break;
    case 'O': pattern = CHAR_O; color = neoColor(150, 80, 0); break;
    case 'L': pattern = CHAR_L; color = neoColor(0, 0, 150); break;
    case 'M': pattern = CHAR_M; color = neoColor(0, 150, 150); break;
    case 'S': pattern = CHAR_S; color = neoColor(120, 0, 150); break;
    case 'D': pattern = CHAR_D; color = neoColor(0, 150, 0); break;
    case 'P': pattern = CHAR_P; color = neoColor(150, 150, 0); break;
    case 'U': pattern = CHAR_U; color = neoColor(150, 150, 150); break;
    default: pattern = CHAR_U; color = neoColor(150, 150, 150); break;
  }

  if (pattern == nullptr) {
    return;
  }

  for (uint8_t row = 0; row < 3; row++) {
    for (uint8_t col = 0; col < 5; col++) {
      uint8_t pi = static_cast<uint8_t>(row * 5 + col);
      if (pattern[pi]) {
        size_t idx = neoMatrixIndex(static_cast<uint8_t>(row + 1), col, auxStartIndex);
        if (idx < frameCount) {
          frame[idx] = color;
        }
      }
    }
  }
}

static void neoRenderTest(uint32_t *frame,
                          uint8_t frameCount,
                          uint8_t auxStartIndex) {
  if (frame == nullptr || frameCount <= auxStartIndex) {
    return;
  }

  uint8_t maxExtra = static_cast<uint8_t>(frameCount - auxStartIndex);
  uint8_t extra = neoTestExtraPixels;
  if (extra > maxExtra) {
    extra = maxExtra;
  }

  uint32_t color = hslToRgb(neoTestHue, neoTestSat, neoTestLight);
  for (uint8_t i = 0; i < extra; i++) {
    frame[auxStartIndex + i] = color;
  }
}

}  // namespace

const char *neoAuxModeName(uint8_t mode) {
  switch (mode) {
    case NEO_AUX_NONE: return "NONE";
    case NEO_AUX_SHIFTLIGHT: return "SHIFT";
    case NEO_AUX_GEAR: return "GEAR";
    case NEO_AUX_TEST: return "TEST";
    default: return "UNKNOWN";
  }
}

void neoModesInit() {
  neoRpm = 0;
  neoRpmUpdatedMs = 0;
  neoGearChar = 'N';
  neoGearUpdatedMs = 0;
  neoTestExtraPixels = NEO_SHIFTLIGHT_COUNT;
  neoTestHue = 0;
  neoTestSat = 100;
  neoTestLight = 40;
}

void neoModesUpdateFromCan(const CanMsg &msg, uint32_t nowMs) {
  if (msg.id == 0x360 && msg.data_length >= 2) {
    neoRpm = static_cast<uint16_t>(msg.data[1] + (static_cast<uint16_t>(msg.data[0]) << 8));
    neoRpmUpdatedMs = nowMs;
    return;
  }
  if (msg.id == 0x600 && msg.data_length >= 2) {
    neoRpm = static_cast<uint16_t>(msg.data[1] + (static_cast<uint16_t>(msg.data[0]) << 8));
    neoRpmUpdatedMs = nowMs;
    return;
  }
  if (msg.id == 0x0A5 && msg.data_length >= 7) {
    uint16_t raw = static_cast<uint16_t>(msg.data[5]) + (static_cast<uint16_t>(msg.data[6]) << 8);
    neoRpm = static_cast<uint16_t>(raw / 4U);
    neoRpmUpdatedMs = nowMs;
    return;
  }
  if (msg.id == 0x470 && msg.data_length >= 8) {
    neoGearChar = decodeHaltechGearByte(msg.data[7]);
    neoGearUpdatedMs = nowMs;
  }
}

void neoModesRender(uint8_t auxMode,
                    uint32_t nowMs,
                    uint32_t *frame,
                    uint8_t frameCount,
                    uint8_t auxStartIndex) {
  switch (auxMode) {
    case NEO_AUX_SHIFTLIGHT:
      neoRenderShiftlight(nowMs, frame, frameCount, auxStartIndex);
      break;
    case NEO_AUX_GEAR:
      neoRenderGear(nowMs, frame, frameCount, auxStartIndex);
      break;
    case NEO_AUX_TEST:
      neoRenderTest(frame, frameCount, auxStartIndex);
      break;
    default:
      break;
  }
}

uint8_t neoModesGetTestExtraPixels() {
  return neoTestExtraPixels;
}

void neoModesSetTestExtraPixels(uint8_t count) {
  neoTestExtraPixels = count;
}

uint16_t neoModesGetTestHue() {
  return neoTestHue;
}

uint8_t neoModesGetTestSat() {
  return neoTestSat;
}

uint8_t neoModesGetTestLight() {
  return neoTestLight;
}

void neoModesSetTestHsl(uint16_t hue, uint8_t sat, uint8_t light) {
  neoTestHue = hue;
  neoTestSat = sat;
  neoTestLight = light;
}
