#ifndef MCP_PRIMITIVES_HPP
#define MCP_PRIMITIVES_HPP
#include "MCP_Constants.hpp"

#include "Utility.hpp"
#include "atomic"
#include <array>
#include <cassert>

namespace MCP {

//
struct Pin {
private:
  const PIN pinEnum;
  const PORT port;
  const uint8_t pinNumber;
  const uint8_t mask;

  bool state;
  bool interruptEnabled;

public:
  // Constructor

  constexpr Pin(PIN pin)
      : pinEnum(pin), port(Util::getPortFromPin(pin)),
        pinNumber(Util::getPinIndex(pin)), mask(1 << (Util::getPinIndex(pin))),
        state(false), interruptEnabled(false) {}

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

  // State management
  bool getState() const { return state; }
  void setState(bool newState) { state = newState; }
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