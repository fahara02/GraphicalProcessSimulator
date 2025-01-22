#ifndef MCP23017_DEFINES_HPP
#define MCP23017_DEFINES_HPP
#include "atomic"
#include "stdint.h"
#include <array>
#include <cassert>

namespace MCP23017 {

static constexpr uint16_t INT_ERR = 255;

static constexpr uint16_t DEFAULT_I2C_ADDRESS = 0x20;
static constexpr MCP_I2C_CLK DEFAULT_I2C_CLK_FRQ = MCP_I2C_CLK::CLK_STD;
static constexpr uint16_t I2C_MASTER_TX_BUF_DISABLE = 0;
static constexpr uint16_t I2C_MASTER_RX_BUF_DISABLE = 0;

static constexpr uint16_t PIN_PER_BANK = 8;
static constexpr uint16_t MAX_PIN = 2 * PIN_PER_BANK;

class BitUtil {
public:
  static void setBit(uint32_t &byte, uint8_t bit) { byte |= (1UL << bit); }

  static void clearBit(uint32_t &byte, uint8_t bit) { byte &= ~(1UL << bit); }

  static bool isBitSet(const uint32_t &byte, uint8_t bit) {
    return (byte & (1UL << bit)) != 0U;
  }
};

enum class PIN {
  PIN0 = 0,
  PIN1 = 1,
  PIN2 = 2,
  PIN3 = 3,
  PIN4 = 4,
  PIN5 = 5,
  PIN6 = 6,
  PIN7 = 7,
  PIN8 = 8,
  PIN9 = 9,
  PIN10 = 10,
  PIN11 = 11,
  PIN12 = 12,
  PIN13 = 13,
  PIN14 = 14,
  PIN15 = 15,
};
enum class MASK {
  NONE = 0x00,
  ALL = 0xFF,
  EVEN = 0x55,       // 01010101
  ODD = 0xAA,        // 10101010
  LOWER_HALF = 0x0F, // 00001111
  UPPER_HALF = 0xF0  // 11110000
};

enum class PORT {
  GPIOA = 1,
  GPIOB = 2,
};
enum class REG {
  IODIRA = 0x00,
  IODIRB,
  IPOLA,
  IPOLB,
  GPINTENA,
  GPINTENB,
  DEFVALA,
  DEFVALB,
  INTCONA,
  INTCONB,
  GPPUA,
  GPPUB,
  INTFA,
  INTFB,
  INTCAPA,
  INTCAPB,
  GPIOA,
  GPIOB,
  OLATA,
  OLATB

};
enum class REG_FUNCTION {
  CONTROL,       // IOCON
  GPIO_DATA,     // GPIO
  GPIO_DIR,      // IODIR
  GPIO_PULL,     // GPPU
  GPIO_LATCH,    // OLA
  GPIO_POLARITY, // IPOLA
  INTR_ENABLE,   // GPINTEN
  INTR_CONTROL,  // INT_CON
  INTR_FLAG,     // INTF

};
enum class PIN_STATE {

  LOW,
  HIGH,
  UNDEFINED
};
enum class GPIO_MODE {
  INPUT = 0,
  INPUT_PULLUP = 2,
  OUTPUT = 3,
  NA

};

enum class PULL_MODE {

  INTERNAL_PULLUP,
  PULL_DOWN,
  NONE

};
enum class INTR_TYPE { CHANGE, FALLING, RISING, NONE };
enum class INTR_OUTPUT_TYPE {
  ACTIVE_HIGH = 0,
  ACTIVE_LOW = 1,
  OPEN_DRAIN = 2,
  NA = 3
};

// MCP DEVICE RELATED
enum class MCP_I2C_CLK {
  CLK_STD = 100000,  // 100KHz
  CLK_HIGH = 400000, // 400 Khz
  CLK_MAX = 17000000 // 1.7 MHZ

};
enum class MCP_MODE { BYTE_MODE = 0, SEQUENTIAL_MODE = 1 };

//

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
  std::atomic<PIN_STATE> state;
  std::atomic<bool> interruptEnabled;

public:
  // Constructor
  constexpr Pin(PIN pin)
      : pinEnum(pin), port(getPortFromPin(pin)),
        pinNumber(static_cast<uint8_t>(pin) % 8),
        mask(1 << (static_cast<uint8_t>(pin) % 8)), mode(GPIO_MODE::INPUT),
        pullMode(PULL_MODE::NONE), interruptType(INTR_TYPE::NONE),
        intrOutputType(INTR_OUTPUT_TYPE::ACTIVE_HIGH), interruptEnabled(false),
        state(PIN_STATE::UNDEFINED) {}
  // Full copy constructor
  Pin(const Pin &other)
      : pinEnum(other.pinEnum), port(other.port), pinNumber(other.pinNumber),
        mask(other.mask), mode(other.mode), pullMode(other.pullMode),
        interruptType(other.interruptType),
        intrOutputType(other.intrOutputType),
        state(other.state.load()), // Copy atomic state
        interruptEnabled(other.interruptEnabled.load()) {}

  // Core constants getters
  constexpr uint8_t getPinNumber() const { return pinNumber; }
  constexpr uint8_t getMask() const { return mask; }

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
    interruptEnabled.store(true);
  }

  INTR_OUTPUT_TYPE getInterruptOutputType() const { return intrOutputType; }

  // State management
  PIN_STATE getState() const { return state.load(); }
  void setState(PIN_STATE newState) { state.store(newState); }

  // Interrupt management
  bool isInterruptEnabled() const { return interruptEnabled.load(); }
  void clearInterrupt() { interruptEnabled.store(false); }

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
      : generalMask(static_cast<uint8_t>(MASK::ALL)), interruptMask(0x00),
        regMap(createRegMap(port, enableInterrupt)), Pins(createPins(port)),
        pinInterruptState{false}, interruptEnabled(enableInterrupt),
        port_name(port), ports(0), ddr(0), pull_ups(0),
        intr_type(INTR_TYPE::NONE), intr_out_type(INTR_OUTPUT_TYPE::NA) {
    assert(isValidPort(port) && "Invalid PORT provided!"); // Validate PORT
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
    assert((mask & ~0xFF) == 0 && "Invalid mask: must be an 8-bit value!");
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
        result |= (Pins[i].getState() == PIN_STATE::HIGH ? (1 << i) : 0);
      }
    }
    return result;
  }
  // Set the pin state by index
  void setPinState(uint8_t index, PIN_STATE state) {
    if (index < PIN_PER_BANK) {
      Pins[index].setState(state);
    }
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
  void setupInterrupt(uint8_t pinMask = 0xFF,
                      IntrType intrType = INTR_TYPE::CHANGE,
                      IntrOutType intrOutType = INTR_OUTPUT_TYPE::ACTIVE_HIGH) {
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
};

// Define GPIO Bank instances
constexpr GPIO_BANKS BANK_A(PORT::GPIOA);
constexpr GPIO_BANKS BANK_B(PORT::GPIOB);

} // namespace MCP23017

#endif