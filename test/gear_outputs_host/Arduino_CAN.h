#pragma once

#include <stdint.h>

struct CanMsg {
  uint32_t id = 0;
  uint8_t data_length = 0;
  uint8_t data[8] = {};
};