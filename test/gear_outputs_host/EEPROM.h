#pragma once

#include <cstring>
#include <stdint.h>

struct TestEeprom {
  uint8_t bytes[1024] = {};

  template <typename Value>
  void get(int address, Value &value) {
    std::memcpy(&value, bytes + address, sizeof(value));
  }

  template <typename Value>
  void put(int address, const Value &value) {
    std::memcpy(bytes + address, &value, sizeof(value));
  }
};

extern TestEeprom EEPROM;