#ifndef UTILITY_HPP
#define UTILITY_HPP
#include "MCP_Constants.hpp"
namespace MCP {

class Util {

public:
  static constexpr PORT getPortFromPin(PIN pin) {
    return (static_cast<uint8_t>(pin) < 8) ? PORT::GPIOA : PORT::GPIOB;
  }
  static constexpr uint8_t getPinIndex(PIN pin) {
    return static_cast<uint8_t>(pin) % 8;
  }
  static constexpr uint8_t getIndexFromPin(PIN pin) {
    uint8_t index = 0;
    PORT port = getPortFromPin(pin);
    uint8_t pinEnum = static_cast<uint8_t>(pin);
    return index = (port == PORT::GPIOB) ? (pinEnum - 8) : pinEnum;
  }

  class BIT {
  public:
    static void set(uint8_t &byte, uint8_t bit) { byte |= (1UL << bit); }
    static void clear(uint8_t &byte, uint8_t bit) { byte &= ~(1UL << bit); }
    static bool isSet(const uint8_t &byte, uint8_t bit) {
      return (byte & (1UL << bit)) != 0U;
    }
  };
};

} // namespace MCP

#endif