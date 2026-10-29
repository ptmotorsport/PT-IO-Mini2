#pragma once

#include <stdint.h>

constexpr uint32_t gearBitForByte(uint8_t gearByte) {
  return gearByte <= 9 ? (1UL << gearByte)
       : gearByte >= 248 ? (1UL << (gearByte - 238U)) : 0UL;
}

constexpr char gearCharForByte(uint8_t gearByte) {
  return gearByte == 0 ? 'N'
       : gearByte <= 9 ? static_cast<char>('0' + gearByte)
       : gearByte == 248 ? 'O' : gearByte == 249 ? 'L'
       : gearByte == 250 ? 'M' : gearByte == 251 ? 'S'
       : gearByte == 252 ? 'D' : gearByte == 253 ? 'U'
       : gearByte == 254 ? 'P' : gearByte == 255 ? 'R' : 'U';
}

constexpr uint32_t gearBitForChar(char gear) {
  return gear == 'N' ? gearBitForByte(0)
       : gear >= '1' && gear <= '9' ? gearBitForByte(gear - '0')
       : gear == 'O' ? gearBitForByte(248) : gear == 'L' ? gearBitForByte(249)
       : gear == 'M' ? gearBitForByte(250) : gear == 'S' ? gearBitForByte(251)
       : gear == 'D' ? gearBitForByte(252) : gear == 'U' ? gearBitForByte(253)
       : gear == 'P' ? gearBitForByte(254) : gear == 'R' ? gearBitForByte(255) : 0UL;
}

constexpr uint32_t defaultGearBinding(uint8_t outputIndex) {
  return outputIndex < 4 ? gearBitForByte(outputIndex + 1U)
       : outputIndex == 4 ? (gearBitForChar('5') | gearBitForChar('D'))
       : outputIndex == 5 ? gearBitForChar('N')
       : outputIndex == 6 ? gearBitForChar('R')
       : outputIndex == 7 ? gearBitForChar('P') : 0UL;
}

constexpr uint8_t gearBindingDuty(uint32_t binding, uint8_t gearByte) {
  return (binding & gearBitForByte(gearByte)) != 0 ? 255 : 0;
}

inline bool parseGearBinding(const char *text, uint32_t &binding) {
  if (text == nullptr || *text == '\0') return false;
  if (text[0] == 'N' && text[1] == 'O' && text[2] == 'N' && text[3] == 'E' && text[4] == '\0') {
    binding = 0;
    return true;
  }
  uint32_t parsed = 0;
  while (*text != '\0') {
    uint32_t bit = gearBitForChar(*text++);
    if (bit == 0) return false;
    parsed |= bit;
    if (*text == '\0') break;
    if (*text++ != '|' || *text == '\0') return false;
  }
  binding = parsed;
  return true;
}