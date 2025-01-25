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
struct GPIO_BANKS {

  std::array<Pin, PIN_PER_BANK> Pins;
  std::array<bool, PIN_PER_BANK> pinInterruptState;

  constexpr GPIO_BANKS(PORT port, bool enableInterrupt = false)
      : Pins(createPins(port)), pinInterruptState{false},
        interruptEnabled(enableInterrupt),
        generalMask(static_cast<uint8_t>(MASK::ALL)), interruptMask(0x00),
        port_name(port), ports(0), ddr(0), pull_ups(0),
        intr_type(INTR_TYPE::NONE), intr_out_type(INTR_OUTPUT_TYPE::NA) {
    assert(isValidPort(port) && "Invalid PORT provided!");
  }
  template <typename... PinType>
  constexpr GPIO_BANKS(PORT port, PinType... pins)
      : Pins(fillPinsArray(pins...)), pinInterruptState{false},
        interruptEnabled(false), generalMask(calculateMask(pins...)),
        interruptMask(0x00), port_name(port), ports(0), ddr(0), pull_ups(0),
        intr_type(INTR_TYPE::NONE), intr_out_type(INTR_OUTPUT_TYPE::NA) {
    // Validate port
    assert(isValidPort(port) && "Invalid PORT provided!");

    // Validate that all pins belong to the same port
    assert(validatePins(port, pins...) &&
           "All pins must belong to the same port!");
  }

  void init() {

    updatePins();
    if (interruptEnabled) {
      updatePinInterruptState();
    }
  }
  // Pin masks
  void setGeneralMask(MASK mask) {
    generalMask = static_cast<uint8_t>(mask);
    updatePins();
    if (interruptEnabled) {
      updatePinInterruptState();
    }
  }

  void setGeneralMask(uint8_t mask) {
    // Ensure the mask is valid for the current configuration of PIN_PER_BANK
    static_assert(PIN_PER_BANK <= 8, "Only 8-bit masks are supported!");

    // Assert that the mask fits within the valid range
    assert((mask & ~((1 << PIN_PER_BANK) - 1)) == 0 &&
           "Invalid mask: must be within the valid pin range!");

    generalMask = mask;
    updatePins();
    if (interruptEnabled) {
      updatePinInterruptState();
    }
  }

  uint8_t getGeneralMask() const { return generalMask; }

  uint8_t getInterruptMask() const { return interruptMask; }

  // Get the pin state by index
  PIN_STATE getPinState(uint8_t index) const {
    if (index >= PIN_PER_BANK) {
      return PIN_STATE::UNDEFINED; // Invalid index
    }
    return Pins[index].getState();
  }
  uint8_t getPinStates() const {
    uint8_t result = 0;
    for (uint8_t i = 0; i < PIN_PER_BANK; ++i) {
      if (generalMask & (1 << i)) { // Check if pin is selected by the mask
        result |= (Pins[i].getState() == PIN_STATE::ON ? (1 << i) : 0);
      }
    }
    return result;
  }
  // Set the pin state by index
  void setPinState(uint8_t index, PIN_STATE state) {
    // Validate PIN_PER_BANK at compile-time
    static_assert(PIN_PER_BANK <= 8, "Invalid PIN_PER_BANK configuration!");

    // Ensure index is within valid range at runtime
    assert(index < PIN_PER_BANK && "Invalid pin index!");

    // Ensure the pin is enabled by the general mask
    if (!(generalMask & (1 << index))) {
      assert(false && "Pin is not enabled by the general mask!");
      return;
    }

    Pins[index].setState(state); // Set the pin state
  }

  void setPinState(PIN_STATE state) {
    for (uint8_t i = 0; i < PIN_PER_BANK; ++i) {
      if (generalMask & (1 << i)) {
        Pins[i].setState(state);
      }
    }
  }

  void setupInterrupt(uint8_t pinMask = 0xFF,
                      INTR_TYPE intrType = INTR_TYPE::NONE,
                      INTR_OUTPUT_TYPE intrOutType = INTR_OUTPUT_TYPE::NA) {
    uint8_t maskToApply = pinMask & generalMask;
    for (uint8_t i = 0; i < PIN_PER_BANK; ++i) {
      if (maskToApply & (1 << i)) {
        intr_type = intrType;
        intr_out_type = intrOutType;

        pinInterruptState[i] = true; // Update interrupt state
      } else {
        pinInterruptState[i] = false;
      }
    }
    setInterruptMask(maskToApply);
  }

  // Set pull-up resistor for a pin by index
  void setPullup(uint8_t index, bool enable) {}

  // Set pin direction (INPUT or OUTPUT) by index
  void setPinDirection(uint8_t index, GPIO_MODE mode) {}

  bool isInterruptEnabled() const { return interruptEnabled; }

private:
  bool interruptEnabled;
  uint8_t generalMask;
  uint8_t interruptMask;
  PORT port_name;
  uint8_t ports;
  uint8_t ddr;
  uint8_t pull_ups;
  INTR_TYPE intr_type;
  INTR_OUTPUT_TYPE intr_out_type;

  void setInterruptMask(uint8_t pinMask) {
    interruptMask = pinMask & 0xFF;
    if (interruptEnabled) {
      updatePinInterruptState();
    }
  }
  void updatePins() {
    for (uint8_t i = 0; i < PIN_PER_BANK; ++i) {
      bool isGeneralMaskSet = BitUtil::isBitSet(generalMask, i);

      if (!isGeneralMaskSet) {

        Pins[i].setState(PIN_STATE::UNDEFINED);
      }
    }
  }

  void updatePinInterruptState() {
    for (uint8_t i = 0; i < PIN_PER_BANK; ++i) {
      bool isGeneralMaskSet = BitUtil::isBitSet(generalMask, i);
      bool isInterruptMaskSet = BitUtil::isBitSet(interruptMask, i);
      pinInterruptState[i] = isGeneralMaskSet && isInterruptMaskSet;
    }
  }

  static constexpr bool isValidPort(PORT port) {
    return port == PORT::GPIOA || port == PORT::GPIOB;
  }

  static constexpr std::array<Pin, PIN_PER_BANK> createPins(PORT port) {
    std::array<Pin, PIN_PER_BANK> pins{};
    for (size_t i = 0; i < PIN_PER_BANK; ++i) {
      pins[i] =
          Pin(static_cast<PIN>(static_cast<uint8_t>(port) * PIN_PER_BANK + i));
    }
    return pins;
  }

  // Helper to calculate the general mask
  template <typename... PinType>
  static constexpr uint8_t calculateMask(PinType... pins) {
    return (0 | ... | pins.getMask());
  }

  // Helper to validate pins belong to the same port
  template <typename... PinType>
  static constexpr bool validatePins(PORT port, PinType... pins) {
    return ((pins.getPort() == port) && ...);
  }
  template <typename... PinType>
  static constexpr std::array<Pin, PIN_PER_BANK>
  fillPinsArray(PinType... pins) {
    std::array<Pin, PIN_PER_BANK> pinArray = {};
    size_t i = 0;

    // Assign provided pins
    ((pinArray[i++] = pins), ...);

    // Fill remaining Pins with default
    while (i < PIN_PER_BANK) {
      pinArray[i++] = Pin();
    }

    return pinArray;
  }
};
// constexpr GPIO_BANKS<MCP23017Pin> gpioBankA(PORT::GPIOA);
// // Define GPIO Bank instances
constexpr GPIO_BANKS BANK_A(PORT::GPIOA);

// Using the variadic constructor
constexpr GPIO_BANKS bankA_custom(PORT::GPIOA, GPA0, GPA1, GPA2);

} // namespace MCP

#endif