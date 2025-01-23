#ifndef MCP_DEFINES_HPP
#define MCP_DEFINES_HPP
#include "MCP_Constants.hpp"
#include "atomic"
#include <array>
#include <cassert>

namespace MCP {

class BitUtil {
public:
  static void setBit(uint32_t &byte, uint8_t bit) { byte |= (1UL << bit); }

  static void clearBit(uint32_t &byte, uint8_t bit) { byte &= ~(1UL << bit); }

  static bool isBitSet(const uint32_t &byte, uint8_t bit) {
    return (byte & (1UL << bit)) != 0U;
  }
};

struct Registe_ICON {
  uint8_t BANK : 1;     //!< Controls how the registers are addressed
  uint8_t MIRROR : 1;   //!< INT Pins Mirror bit
  uint8_t SEQOP : 1;    //!< Sequential Operation mode bit
  uint8_t DISSLW : 1;   //!< Slew Rate control bit for SDA output
  uint8_t HAEN : 1;     //!< Enables hardware addressing
  uint8_t ODR : 1;      //!< Configures the INT pin as an open-drain output
  uint8_t INTPOL : 1;   //!< Sets the polarity of the INT output pin
  uint8_t RESERVED : 1; //!< Reserved bit (unused)

  // Function to create the configuration byte with default arguments
  static uint8_t createConfig(
      MCP_BANK_MODE bank = MCP_BANK_MODE::MERGE_BANK,
      MCP_MIRROR_MODE mirror = MCP_MIRROR_MODE::INT_DISCONNECTED,
      MCP_OPERATION_MODE seqop = MCP_OPERATION_MODE::SEQUENTIAL_MODE,
      MCP_SLEW_RATE disslew = MCP_SLEW_RATE::SLEW_ENABLED,
      MCP_HARDWARE_ADDRESSING haen = MCP_HARDWARE_ADDRESSING::HAEN_DISABLED,
      MCP_OPEN_DRAIN odr = MCP_OPEN_DRAIN::ACTIVE_DRIVER,
      MCP_INT_POL intpol = MCP_INT_POL::MCP_ACTIVE_LOW) {
    Registe_ICON reg = {
        static_cast<uint8_t>(bank),
        static_cast<uint8_t>(mirror),
        static_cast<uint8_t>(seqop),
        static_cast<uint8_t>(disslew),
        static_cast<uint8_t>(haen),
        static_cast<uint8_t>(odr),
        static_cast<uint8_t>(intpol),
        0 // RESERVED
    };

    return *reinterpret_cast<uint8_t *>(&reg);
  }
};

struct Pin {
  const PIN pinEnum;
  const PORT port;
  const uint8_t pinNumber;
  const uint8_t mask;

private:
  GPIO_MODE mode;
  PULL_MODE pullMode;
  INTR_TYPE interruptType;
  INTR_OUTPUT_TYPE intrOutputType;
  PIN_STATE state;
  bool interruptEnabled;

public:
  // Constructor

  constexpr Pin(PIN pin)
      : pinEnum(pin), port(getPortFromPin(pin)),
        pinNumber(static_cast<uint8_t>(pin) % 8),
        mask(1 << (static_cast<uint8_t>(pin) % 8)), mode(GPIO_MODE::GPIO_INPUT),
        pullMode(PULL_MODE::NONE), interruptType(INTR_TYPE::NONE),
        intrOutputType(INTR_OUTPUT_TYPE::INTR_ACTIVE_HIGH),
        state(PIN_STATE::UNDEFINED), interruptEnabled(false) {}
  constexpr Pin() : Pin(PIN::PIN0) {}
  // Full copy constructor
  constexpr Pin(const Pin &other)
      : pinEnum(other.pinEnum), port(other.port), pinNumber(other.pinNumber),
        mask(other.mask), mode(other.mode), pullMode(other.pullMode),
        interruptType(other.interruptType),
        intrOutputType(other.intrOutputType),
        state(other.state), // Copy atomic state
        interruptEnabled(false) {}
  constexpr Pin &operator=(const Pin &other) {
    if (this != &other) {
      mode = other.mode;
      pullMode = other.pullMode;
      interruptType = other.interruptType;
      intrOutputType = other.intrOutputType;
      interruptEnabled = other.interruptEnabled;
      state = other.state;
    }
    return *this;
  }

  // Core constants getters
  constexpr uint8_t getPinNumber() const { return pinNumber; }
  constexpr uint8_t getMask() const { return mask; }
  constexpr PORT getPort() const { return port; }

  // Pin mode
  GPIO_MODE getMode() const { return mode; }
  void setMode(GPIO_MODE newMode) { mode = newMode; }

  // Pull mode
  PULL_MODE getPullMode() const { return pullMode; }
  void setPullMode(PULL_MODE newPullMode) { pullMode = newPullMode; }

  // Interrupt settings
  INTR_TYPE getInterruptType() const { return interruptType; }
  void setInterrupt(INTR_TYPE newInterruptType,
                    INTR_OUTPUT_TYPE newIntrOutputType) {
    interruptType = newInterruptType;
    intrOutputType = newIntrOutputType;
    interruptEnabled = true;
  }

  INTR_OUTPUT_TYPE getInterruptOutputType() const { return intrOutputType; }

  // State management
  PIN_STATE getState() const { return state; }
  void setState(PIN_STATE newState) { state = newState; }

  // Interrupt management
  bool isInterruptEnabled() const { return interruptEnabled; }
  void clearInterrupt() { interruptEnabled = false; }

private:
  constexpr PORT getPortFromPin(PIN pin) const {
    return (static_cast<uint8_t>(pin) < 8) ? PORT::GPIOA : PORT::GPIOB;
  }
};

// Pin definitions as constexpr
constexpr Pin GPB0 = Pin(PIN::PIN8);
constexpr Pin GPB1 = Pin(PIN::PIN9);
constexpr Pin GPB2 = Pin(PIN::PIN10);
constexpr Pin GPB3 = Pin(PIN::PIN11);
constexpr Pin GPB4 = Pin(PIN::PIN12);
constexpr Pin GPB5 = Pin(PIN::PIN13);
constexpr Pin GPB6 = Pin(PIN::PIN14);
constexpr Pin GPB7 = Pin(PIN::PIN15);

constexpr Pin GPA0 = Pin(PIN::PIN0);
constexpr Pin GPA1 = Pin(PIN::PIN1);
constexpr Pin GPA2 = Pin(PIN::PIN2);
constexpr Pin GPA3 = Pin(PIN::PIN3);
constexpr Pin GPA4 = Pin(PIN::PIN4);
constexpr Pin GPA5 = Pin(PIN::PIN5);
constexpr Pin GPA6 = Pin(PIN::PIN6);
constexpr Pin GPA7 = Pin(PIN::PIN7);

struct Registers {
  REG regName;
  REG_FUNCTION function;

  constexpr Registers(REG reg, REG_FUNCTION func)
      : regName(reg), function(func) {}
};

struct GPIO_BANKS {

  std::array<Registers, 9> regMap;
  std::array<Pin, PIN_PER_BANK> Pins;
  std::array<bool, PIN_PER_BANK> pinInterruptState;

  constexpr GPIO_BANKS(PORT port, bool enableInterrupt = false)
      : regMap(createRegMap(port, enableInterrupt)), Pins(createPins(port)),
        pinInterruptState{false}, interruptEnabled(enableInterrupt),
        generalMask(static_cast<uint8_t>(MASK::ALL)), interruptMask(0x00),
        port_name(port), ports(0), ddr(0), pull_ups(0),
        intr_type(INTR_TYPE::NONE), intr_out_type(INTR_OUTPUT_TYPE::NA) {
    assert(isValidPort(port) && "Invalid PORT provided!"); // Validate PORT
  }
  template <typename... PinsType>
  constexpr GPIO_BANKS(PORT port, PinsType... pins)
      : regMap(createRegMap(port, false)), Pins(fillPinsArray(pins...)),
        pinInterruptState{false}, interruptEnabled(false),
        generalMask(calculateMask(pins...)), interruptMask(0x00),
        port_name(port), ports(0), ddr(0), pull_ups(0),
        intr_type(INTR_TYPE::NONE), intr_out_type(INTR_OUTPUT_TYPE::NA) {
    // Validate port
    assert(isValidPort(port) && "Invalid PORT provided!");

    // Validate that all pins belong to the same port
    static_assert(validatePins(port, pins...),
                  "All pins must belong to the same port!");
  }

  void init() {
    initInternalState();
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
  template <typename IntrType = INTR_TYPE,
            typename IntrOutType = INTR_OUTPUT_TYPE>
  void
  setupInterrupt(uint8_t pinMask = 0xFF,
                 IntrType intrType = INTR_TYPE::INTR_CHANGE,
                 IntrOutType intrOutType = INTR_OUTPUT_TYPE::INTR_ACTIVE_HIGH) {
    uint8_t maskToApply = pinMask & generalMask;
    for (uint8_t i = 0; i < PIN_PER_BANK; ++i) {
      if (maskToApply & (1 << i)) {
        intr_type = intrType;
        intr_out_type = intrOutType;
        Pins[i].setInterrupt(intrType, intrOutType);
        pinInterruptState[i] = true; // Update interrupt state
      } else {
        pinInterruptState[i] = false;
      }
    }
    setInterruptMask(maskToApply);
  }

  // Set pull-up resistor for a pin by index
  void setPullup(uint8_t index, bool enable) {
    if (index < PIN_PER_BANK) {
      Pins[index].setPullMode(enable ? PULL_MODE::INTERNAL_PULLUP
                                     : PULL_MODE::NONE);
    }
  }

  // Set pin direction (INPUT or OUTPUT) by index
  void setPinDirection(uint8_t index, GPIO_MODE mode) {
    if (index < PIN_PER_BANK) {
      Pins[index].setMode(mode);
    }
  }

  // Retrieve a register based on its function
  constexpr REG getRegister(REG_FUNCTION func) const {
    for (const auto &reg : regMap) {
      if (reg.function == func) {
        return reg.regName;
      }
    }
    return static_cast<REG>(0xFF); // Invalid register
  }
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
  void constexpr initInternalState() {
    if (port_name == PORT::GPIOA) {
      ports = 0x01;
      ddr = 0x02;
      pull_ups = 0x04;
    } else if (port_name == PORT::GPIOB) {
      ports = 0x08; // Example initialization for GPIOB
      ddr = 0x10;
      pull_ups = 0x20;
    }
  }
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
        // Reset pin state if excluded by the general mask
        Pins[i].setMode(GPIO_MODE::NA);
        Pins[i].setPullMode(PULL_MODE::NONE);
        Pins[i].clearInterrupt();
        Pins[i].setState(PIN_STATE::UNDEFINED);
      }
    }
  }

  void updatePinInterruptState() {
    for (uint8_t i = 0; i < PIN_PER_BANK; ++i) {
      bool isGeneralMaskSet = BitUtil::isBitSet(generalMask, i);
      bool isInterruptMaskSet = BitUtil::isBitSet(interruptMask, i);
      pinInterruptState[i] = isGeneralMaskSet && isInterruptMaskSet;

      // Update the corresponding pin's interrupt enabled state
      Pins[i].clearInterrupt(); // Reset first
      if (pinInterruptState[i]) {
        Pins[i].setInterrupt(intr_type, intr_out_type);
      }
    }
  }

  static constexpr bool isValidPort(PORT port) {
    return port == PORT::GPIOA || port == PORT::GPIOB;
  }
  static constexpr std::array<Pin, PIN_PER_BANK> createPins(PORT bankPort) {
    return {
        Pin(static_cast<PIN>(0 + (bankPort == PORT::GPIOB ? 8 : 0))),
        Pin(static_cast<PIN>(1 + (bankPort == PORT::GPIOB ? 8 : 0))),
        Pin(static_cast<PIN>(2 + (bankPort == PORT::GPIOB ? 8 : 0))),
        Pin(static_cast<PIN>(3 + (bankPort == PORT::GPIOB ? 8 : 0))),
        Pin(static_cast<PIN>(4 + (bankPort == PORT::GPIOB ? 8 : 0))),
        Pin(static_cast<PIN>(5 + (bankPort == PORT::GPIOB ? 8 : 0))),
        Pin(static_cast<PIN>(6 + (bankPort == PORT::GPIOB ? 8 : 0))),
        Pin(static_cast<PIN>(7 + (bankPort == PORT::GPIOB ? 8 : 0))),
    };
  }

  static constexpr std::array<Registers, 9> createRegMap(PORT port,
                                                         bool enableInterrupt) {
    return {
        Registers(port == PORT::GPIOA ? REG::IODIRA : REG::IODIRB,
                  REG_FUNCTION::GPIO_DIR),
        Registers(port == PORT::GPIOA ? REG::IPOLA : REG::IPOLB,
                  REG_FUNCTION::GPIO_POLARITY),
        Registers(port == PORT::GPIOA ? REG::GPINTENA : REG::GPINTENB,
                  REG_FUNCTION::INTR_ENABLE),
        Registers(port == PORT::GPIOA ? REG::DEFVALA : REG::DEFVALB,
                  REG_FUNCTION::INTR_CONTROL),
        Registers(port == PORT::GPIOA ? REG::INTCONA : REG::INTCONB,
                  REG_FUNCTION::INTR_CONTROL),
        Registers(port == PORT::GPIOA ? REG::GPPUA : REG::GPPUB,
                  REG_FUNCTION::GPIO_PULL),
        Registers(port == PORT::GPIOA ? REG::INTFA : REG::INTFB,
                  REG_FUNCTION::INTR_FLAG),
        Registers(port == PORT::GPIOA ? REG::GPIOA : REG::GPIOB,
                  REG_FUNCTION::GPIO_DATA),
        Registers(port == PORT::GPIOA ? REG::OLATA : REG::OLATB,
                  REG_FUNCTION::GPIO_LATCH),
    };
  }
  // Helper to calculate the general mask
  template <typename... PinsType>
  static constexpr uint8_t calculateMask(PinsType... pins) {
    return (0 | ... | pins.getMask());
  }

  // Helper to validate pins belong to the same port
  template <typename... PinsType>
  static constexpr bool validatePins(PORT port, PinsType... pins) {
    return ((pins.getPort() == port) && ...);
  }
  template <typename... PinsType>
  static constexpr std::array<Pin, PIN_PER_BANK>
  fillPinsArray(PinsType... pins) {
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

// Define GPIO Bank instances
constexpr GPIO_BANKS BANK_A(PORT::GPIOA);
constexpr GPIO_BANKS BANK_B(PORT::GPIOB);

// Using the variadic constructor
// constexpr GPIO_BANKS bankA_custom(PORT::GPIOA, GPA0, GPA1, GPA2);

} // namespace MCP

#endif