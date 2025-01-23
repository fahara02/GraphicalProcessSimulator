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

struct register_icon_t {
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
    register_icon_t reg = {
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

template <typename PinEnum> //
struct Pin {
  using PinEnumType = PinEnum;
  const PinEnum pinEnum;
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

  constexpr Pin(PinEnum pin)
      : pinEnum(pin), port(getPortFromPin(pin)),
        pinNumber(static_cast<uint8_t>(pin) % 8),
        mask(1 << (static_cast<uint8_t>(pin) % 8)), mode(GPIO_MODE::GPIO_INPUT),
        pullMode(PULL_MODE::NONE), interruptType(INTR_TYPE::NONE),
        intrOutputType(INTR_OUTPUT_TYPE::INTR_ACTIVE_HIGH),
        state(PIN_STATE::UNDEFINED), interruptEnabled(false) {}
  constexpr Pin() : Pin(static_cast<PinEnum>(0)) {}
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
  constexpr PORT getPortFromPin(PinEnum pin) const {
    return (static_cast<uint8_t>(pin) < 8) ? PORT::GPIOA : PORT::GPIOB;
  }
};

struct MCP23017 {
  using PinType = Pin<MCP_23X17::PIN>;
};

using MCP23017Pin = MCP23017::PinType;

// Define pins as constexpr
constexpr MCP23017Pin GPB0(MCP_23X17::PIN::PIN8);
constexpr MCP23017Pin GPB1(MCP_23X17::PIN::PIN9);
// constexpr MCP23017Pin GPB2 = MCP23017Pin(MCP_23017::PIN::PIN10);
// constexpr MCP23017Pin GPB3 = MCP23017Pin(MCP_23017::PIN::PIN11);
// constexpr MCP23017Pin GPB4 = MCP23017Pin(MCP_23017::PIN::PIN12);
// constexpr MCP23017Pin GPB5 = MCP23017Pin(MCP_23017::PIN::PIN13);
// constexpr MCP23017Pin GPB6 = MCP23017Pin(MCP_23017::PIN::PIN14);
// constexpr MCP23017Pin GPB7 = MCP23017Pin(MCP_23017::PIN::PIN15);

// constexpr MCP23017Pin GPA0 = MCP23017Pin(MCP_23017::PIN::PIN0);
// constexpr MCP23017Pin GPA1 = MCP23017Pin(MCP_23017::PIN::PIN1);
// constexpr MCP23017Pin GPA2 = MCP23017Pin(MCP_23017::PIN::PIN2);
// constexpr MCP23017Pin GPA3 = MCP23017Pin(MCP_23017::PIN::PIN3);
// constexpr MCP23017Pin GPA4 = MCP23017Pin(MCP_23017::PIN::PIN4);
// constexpr MCP23017Pin GPA5 = MCP23017Pin(MCP_23017::PIN::PIN5);
// constexpr MCP23017Pin GPA6 = MCP23017Pin(MCP_23017::PIN::PIN6);
// constexpr MCP23017Pin GPA7 = MCP23017Pin(MCP_23017::PIN::PIN7);

struct Registers {
  REG regName;
  REG_FUNCTION function;

  constexpr Registers(REG reg, REG_FUNCTION func)
      : regName(reg), function(func) {}
};
template <typename PinType> //
struct GPIO_BANKS {

  std::array<Registers, 9> regMap;
  std::array<PinType, PIN_PER_BANK> allPins;
  std::array<bool, PIN_PER_BANK> pinInterruptState;

  constexpr GPIO_BANKS(PORT port, bool enableInterrupt = false)
      : regMap(createRegMap(port, enableInterrupt)), allPins(createPins(port)),
        pinInterruptState{false}, interruptEnabled(enableInterrupt),
        generalMask(static_cast<uint8_t>(MASK::ALL)), interruptMask(0x00),
        port_name(port), ports(0), ddr(0), pull_ups(0),
        intr_type(INTR_TYPE::NONE), intr_out_type(INTR_OUTPUT_TYPE::NA) {
    assert(isValidPort(port) && "Invalid PORT provided!"); // Validate PORT
  }
  template <typename... Pins>
  constexpr GPIO_BANKS(PORT port, Pins... pins)
      : regMap(createRegMap(port, false)), allPins(fillPinsArray(pins...)),
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
    return allPins[index].getState();
  }
  uint8_t getPinStates() const {
    uint8_t result = 0;
    for (uint8_t i = 0; i < PIN_PER_BANK; ++i) {
      if (generalMask & (1 << i)) { // Check if pin is selected by the mask
        result |= (allPins[i].getState() == PIN_STATE::ON ? (1 << i) : 0);
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

    allPins[index].setState(state); // Set the pin state
  }

  void setPinState(PIN_STATE state) {
    for (uint8_t i = 0; i < PIN_PER_BANK; ++i) {
      if (generalMask & (1 << i)) {
        allPins[i].setState(state);
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
        allPins[i].setInterrupt(intrType, intrOutType);
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
      allPins[index].setPullMode(enable ? PULL_MODE::INTERNAL_PULLUP
                                        : PULL_MODE::NONE);
    }
  }

  // Set pin direction (INPUT or OUTPUT) by index
  void setPinDirection(uint8_t index, GPIO_MODE mode) {
    if (index < PIN_PER_BANK) {
      allPins[index].setMode(mode);
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
        allPins[i].setMode(GPIO_MODE::NA);
        allPins[i].setPullMode(PULL_MODE::NONE);
        allPins[i].clearInterrupt();
        allPins[i].setState(PIN_STATE::UNDEFINED);
      }
    }
  }

  void updatePinInterruptState() {
    for (uint8_t i = 0; i < PIN_PER_BANK; ++i) {
      bool isGeneralMaskSet = BitUtil::isBitSet(generalMask, i);
      bool isInterruptMaskSet = BitUtil::isBitSet(interruptMask, i);
      pinInterruptState[i] = isGeneralMaskSet && isInterruptMaskSet;

      // Update the corresponding pin's interrupt enabled state
      allPins[i].clearInterrupt(); // Reset first
      if (pinInterruptState[i]) {
        allPins[i].setInterrupt(intr_type, intr_out_type);
      }
    }
  }

  static constexpr bool isValidPort(PORT port) {
    return port == PORT::GPIOA || port == PORT::GPIOB;
  }

  static constexpr std::array<PinType, PIN_PER_BANK> createPins(PORT port) {
    std::array<PinType, PIN_PER_BANK> pins{};
    for (size_t i = 0; i < PIN_PER_BANK; ++i) {
      pins[i] = PinType(static_cast<typename PinType::PinEnumType>(
          static_cast<uint8_t>(port) * PIN_PER_BANK + i));
    }
    return pins;
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
  template <typename... Pins>
  static constexpr uint8_t calculateMask(Pins... pins) {
    return (0 | ... | pins.getMask());
  }

  // Helper to validate pins belong to the same port
  template <typename... Pins>
  static constexpr bool validatePins(PORT port, Pins... pins) {
    return ((pins.getPort() == port) && ...);
  }
  template <typename... Pins>
  static constexpr std::array<PinType, PIN_PER_BANK>
  fillPinsArray(Pins... pins) {
    std::array<PinType, PIN_PER_BANK> pinArray = {};
    size_t i = 0;

    // Assign provided pins
    ((pinArray[i++] = pins), ...);

    // Fill remaining Pins with default
    while (i < PIN_PER_BANK) {
      pinArray[i++] = PinType();
    }

    return pinArray;
  }
};
constexpr GPIO_BANKS<MCP23017Pin> gpioBankA(PORT::GPIOA);
// // Define GPIO Bank instances
// constexpr GPIO_BANKS BANK_A(PORT::GPIOA);
// constexpr GPIO_BANKS BANK_B(PORT::GPIOB);

// Using the variadic constructor
// constexpr GPIO_BANKS bankA_custom(PORT::GPIOA, GPA0, GPA1, GPA2);

} // namespace MCP

#endif