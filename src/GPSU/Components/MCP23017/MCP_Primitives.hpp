#ifndef MCP_PRIMITIVES_HPP
#define MCP_PRIMITIVES_HPP
#include "MCP_Constants.hpp"

#include "atomic"
#include <array>
#include <cassert>

namespace MCP {

class BitUtil {
public:
  static void setBit(uint8_t &byte, uint8_t bit) { byte |= (1UL << bit); }

  static void clearBit(uint8_t &byte, uint8_t bit) { byte &= ~(1UL << bit); }
  static bool isBitSet(const uint8_t &byte, uint8_t bit) {
    return (byte & (1UL << bit)) != 0U; // Returns true if bit is 1
  }

  template <typename T> static void setBit(T &value, uint8_t bit) {
    value |= (static_cast<T>(1) << bit);
  }

  template <typename T> static void clearBit(T &value, uint8_t bit) {
    value &= ~(static_cast<T>(1) << bit);
  }

  template <typename T> static bool isBitSet(const T &value, uint8_t bit) {
    return (value & (static_cast<T>(1) << bit)) != 0;
  }
};

//
struct Pin {

  const PIN pinEnum;
  const PORT port;
  const uint8_t pinNumber;
  const uint8_t mask;

private:
  PIN_STATE state;
  bool interruptEnabled;

public:
  // Constructor

  constexpr Pin(PIN pin)
      : pinEnum(pin), port(getPortFromPin(pin)),
        pinNumber(static_cast<uint8_t>(pin) % 8),
        mask(1 << (static_cast<uint8_t>(pin) % 8)), state(PIN_STATE::UNDEFINED),
        interruptEnabled(false) {}

  constexpr Pin() : Pin(static_cast<PIN>(0)) {}

  // Full copy constructor
  constexpr Pin(const Pin &other)
      : pinEnum(other.pinEnum), port(other.port), pinNumber(other.pinNumber),
        mask(other.mask), state(other.state), // Copy atomic state
        interruptEnabled(false) {}

  constexpr Pin &operator=(const Pin &other) {
    if (this != &other) {

      interruptEnabled = other.interruptEnabled;
      state = other.state;
    }
    return *this;
  }

  // Core constants getters
  constexpr uint8_t getPinNumber() const { return pinNumber; }
  constexpr uint8_t getMask() const { return mask; }
  constexpr PORT getPort() const { return port; }
  constexpr uint8_t getIndexFromPin(PIN pin) const {
    uint8_t index = 0;
    PORT port = getPortFromPin(pin);
    uint8_t pinEnum = static_cast<uint8_t>(pin);
    return index = (port == PORT::GPIOB) ? (pinEnum - 8) : pinEnum;
  }

  // State management
  PIN_STATE getState() const { return state; }
  void setState(PIN_STATE newState) { state = newState; }

private:
  constexpr PORT getPortFromPin(PIN pin) const {
    return (static_cast<uint8_t>(pin) < 8) ? PORT::GPIOA : PORT::GPIOB;
  }
};

// GPA Pins (PIN0 to PIN7)
constexpr Pin GPA0(MCP::PIN::PIN0);
constexpr Pin GPA1(MCP::PIN::PIN1);
constexpr Pin GPA2(MCP::PIN::PIN2);
constexpr Pin GPA3(MCP::PIN::PIN3);
constexpr Pin GPA4(MCP::PIN::PIN4);
constexpr Pin GPA5(MCP::PIN::PIN5);
constexpr Pin GPA6(MCP::PIN::PIN6);
constexpr Pin GPA7(MCP::PIN::PIN7);

// GPB Pins (PIN8 to PIN15)
constexpr Pin GPB0(MCP::PIN::PIN8);
constexpr Pin GPB1(MCP::PIN::PIN9);
constexpr Pin GPB2(MCP::PIN::PIN10);
constexpr Pin GPB3(MCP::PIN::PIN11);
constexpr Pin GPB4(MCP::PIN::PIN12);
constexpr Pin GPB5(MCP::PIN::PIN13);
constexpr Pin GPB6(MCP::PIN::PIN14);
constexpr Pin GPB7(MCP::PIN::PIN15);

//

} // namespace MCP

#endif