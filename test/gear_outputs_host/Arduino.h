#pragma once

#include <stdint.h>
#include <string>
#include <cstring>

class String : public std::string {
public:
  using std::string::string;
};

uint32_t millis();