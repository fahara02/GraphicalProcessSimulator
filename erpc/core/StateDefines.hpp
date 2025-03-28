// StateDefines.hpp
#pragma once
#include "stdint.h"
enum class Mode : uint8_t { AUTO, MANUAL };
enum class State : uint8_t {
  INIT = 0,
  RED_STATE,
  GREEN_STATE,
  YELLOW_STATE,
  SYSTEM_FAULT
};
struct Context {
  State curr;
  State prev Mode mode = Mode::AUTO;
};