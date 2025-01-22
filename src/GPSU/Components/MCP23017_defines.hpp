#ifndef MCP23017_DEFINES_HPP
#define MCP23017_DEFINES_HPP
#include "atomic"
#include "stdint.h"
#include <array>

namespace MCP23017 {

static constexpr uint16_t INT_ERR = 255;
static constexpr uint16_t DEFAULT_I2C_ADDRESS = 0x20;
static constexpr MCP_I2C_CLK DEFAULT_I2C_CLK_FRQ = MCP_I2C_CLK::CLK_STD;
static constexpr uint16_t I2C_MASTER_TX_BUF_DISABLE = 0;
static constexpr uint16_t I2C_MASTER_RX_BUF_DISABLE = 0;

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
enum class PIN_STATE {

  LOW,
  HIGH,
  UNDEFINED
};
enum class GPIO_MODE {
  INPUT = 0,
  INPUT_PULLUP = 2,
  OUTPUT = 3

};

enum class PULL_MODE {

  INTERNAL_PULLUP,
  PULL_DOWN,
  NONE

};
enum class INTR_TYPE { CHANGE, FALLING, RISING, NONE };
enum class INTR_OUTPUT_TYPE { ACTIVE_HIGH = 0, ACTIVE_LOW = 1, OPEN_DRAIN = 2 };
enum class MCP_I2C_CLK {
  CLK_STD = 100000,  // 100KHz
  CLK_HIGH = 400000, // 400 Khz
  CLK_MAX = 17000000 // 1.7 MHZ

};
enum class MCP_MODE { BYTE_MODE = 0, SEQUENTIAL_MODE = 1 };

struct Pin {
  PIN pinEnum;       // The pin (e.g., PIN0, PIN1, etc.)
  PORT port;         // Associated port (GPIOA or GPIOB)
  uint8_t pinNumber; // Pin number (0-7 for GPIOA and GPIOB)
  uint8_t mask;      // Pin bitmask (1 << pin number)

private:
  // Pin configuration settings
  GPIO_MODE mode;          // Pin mode (INPUT, OUTPUT, INPUT_PULLUP)
  PULL_MODE pullMode;      // Pull-up/Pull-down mode
  INTR_TYPE interruptType; // Interrupt type (CHANGE, FALLING, RISING, NONE)
  INTR_OUTPUT_TYPE intrOutputType; // Interrupt output type
  std::atomic<PIN_STATE> state;    // Atomic pin state (LOW/HIGH)
  bool interruptEnabled;           // Interrupt enable flag

public:
  // Constructor
  constexpr Pin(PIN pin)
      : pinEnum(pin), port(getPortFromPin(pin)),
        pinNumber(static_cast<uint8_t>(pin) % 8),   // Get the pin number (0-7)
        mask(1 << (static_cast<uint8_t>(pin) % 8)), // Bitmask (1 << pin)
        mode(GPIO_MODE::INPUT),                     // Default mode is INPUT
        pullMode(PULL_MODE::NONE),                  // Default pull mode is NONE
        interruptType(INTR_TYPE::NONE), // Default interrupt type is NONE
        intrOutputType(
            INTR_OUTPUT_TYPE::ACTIVE_HIGH), // Default interrupt output type is
                                            // ACTIVE_HIGH
        interruptEnabled(false),            // Default interrupt is disabled
        state(PIN_STATE::UNDEFINED) {}      // Default state is UNDEFINED

  // Getter and Setter functions
  GPIO_MODE getMode() const { return mode; }
  void setMode(GPIO_MODE newMode) { mode = newMode; }

  PULL_MODE getPullMode() const { return pullMode; }
  void setPullMode(PULL_MODE newPullMode) { pullMode = newPullMode; }

  INTR_TYPE getInterruptType() const { return interruptType; }
  void setInterrupt(INTR_TYPE newInterruptType,
                    INTR_OUTPUT_TYPE newIntrOutputType) {
    interruptType = newInterruptType;
    intrOutputType = newIntrOutputType;
    interruptEnabled = true;
  }

  INTR_OUTPUT_TYPE getInterruptOutputType() const { return intrOutputType; }

  PIN_STATE getState() const { return state.load(); } // Read atomic state
  void setState(PIN_STATE newState) {
    state.store(newState);
  } // Write atomic state

  bool isInterruptEnabled() const { return interruptEnabled; }
  void clearInterrupt() { interruptEnabled = false; }

private:
  // Helper to determine the port based on the pin number
  constexpr PORT getPortFromPin(PIN pin) const {
    if (static_cast<uint8_t>(pin) < 8) {
      return PORT::GPIOA; // Pins 0-7 belong to GPIOA
    } else {
      return PORT::GPIOB; // Pins 8-15 belong to GPIOB
    }
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

struct GPIO_BANKS {
  uint8_t mask;  // Bitmask for the pins (e.g., 0xFF for all pins active)
  REG portReg;   // Register for the port (e.g., GPIOA or GPIOB)
  REG pullupReg; // Register for pull-up configuration
  REG dirReg;    // Register for direction (input/output)

  // Constructor to initialize GPIO_BANKS
  constexpr GPIO_BANKS(REG port, REG pullup, REG dir,
                       uint8_t initialMask = 0x00)
      : mask(initialMask), portReg(port), pullupReg(pullup), dirReg(dir) {}
};

} // namespace MCP23017

#endif