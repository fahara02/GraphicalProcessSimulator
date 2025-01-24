#ifndef MCP_CONSTANTS_HPP
#define MCP_CONSTANTS_HPP
#include "stdint.h"
namespace MCP {

// MCP DEVICE RELATED
enum class MCP_MODEL {
  MCP23017,
  MCP23S17,
  MCP23018,
  MCP23S18

};

namespace MCP_23X17 {

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
enum class REG : uint8_t {
  IODIR = 0x00, //!< I/O Direction Register
  IPOL,         //!< Input Polarity Register
  GPINTEN,      //!< Interrupt-on-Change Enable Register
  DEFVAL,       //!< Default Compare Register
  INTCON,       //!< Interrupt Control Register
  IOCON,        //!< Configuration Register
  GPPU,         //!< Pull-Up Resistor Register
  INTF,         //!< Interrupt Flag Register
  INTCAP,       //!< Interrupt Captured Value Register
  GPIO,         //!< General Purpose I/O Register
  OLAT          //!< Output Latch Register
};

namespace MCP_23017 {
enum class I2C_CLK {
  CLK_STD = 100000,  // 100KHz
  CLK_HIGH = 400000, // 400 Khz
  CLK_MAX = 17000000 // 1.7 MHZ

};

};                     // namespace MCP_23017
namespace MCP_23S17 {} // namespace MCP_23S17

}; // namespace MCP_23X17

namespace MCP_23X18 {
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
enum class REG : uint8_t {
  IODIR = 0x00, //!< I/O Direction Register
  IPOL,         //!< Input Polarity Register
  GPINTEN,      //!< Interrupt-on-Change Enable Register
  DEFVAL,       //!< Default Compare Register
  INTCON,       //!< Interrupt Control Register
  IOCON,        //!< Configuration Register
  GPPU,         //!< Pull-Up Resistor Register
  INTF,         //!< Interrupt Flag Register
  INTCAP,       //!< Interrupt Captured Value Register
  GPIO,         //!< General Purpose I/O Register
  OLAT          //!< Output Latch Register
};
}; // namespace MCP_23X18

// Enums for the configuration options
enum class MCP_BANK_MODE { MERGE_BANK = 0, SEPARATE_BANK = 1 };
enum class MCP_MIRROR_MODE { INT_DISCONNECTED = 0, INT_CONNECTED = 1 };
enum class MCP_OPERATION_MODE { SEQUENTIAL_MODE = 0, BYTE_MODE = 1 };
enum class MCP_SLEW_RATE { SLEW_ENABLED = 0, SLEW_DISABLED = 1 };
enum class MCP_HARDWARE_ADDRESSING { HAEN_DISABLED = 0, HAEN_ENABLED = 1 };
enum class MCP_OPEN_DRAIN { ACTIVE_DRIVER = 0, ODR = 1 };
enum class MCP_INT_POL { MCP_ACTIVE_LOW = 0, MCP_ACTIVE_HIGH = 1 };

static constexpr uint16_t INT_ERR = 255;

static constexpr uint16_t DEFAULT_I2C_ADDRESS = 0x20;
static constexpr uint16_t DEFAULT_I2C_CLK_FRQ =
    static_cast<uint16_t>(MCP_23X17::MCP_23017::I2C_CLK::CLK_STD);
static constexpr uint16_t I2C_MASTER_TX_BUF_DISABLE = 0;
static constexpr uint16_t I2C_MASTER_RX_BUF_DISABLE = 0;

static constexpr uint16_t PIN_PER_BANK = 8;
static constexpr uint16_t MAX_PIN = 2 * PIN_PER_BANK;
static constexpr uint16_t MAX_REG_PER_PORT = 11;
static constexpr uint16_t MAX_REG_PER_DEVICE = 2 * MAX_REG_PER_PORT;

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
enum class REG : uint8_t {
  IODIRA = 0x00,   //!< I/O Direction Register A
  IODIRB = 0x01,   //!< I/O Direction Register B
  IPOLA = 0x02,    //!< Input Polarity Port Register A
  IPOLB = 0x03,    //!< Input Polarity Port Register B
  GPINTENA = 0x04, //!< Interrupt-on-Change Enable Register A
  GPINTENB = 0x05, //!< Interrupt-on-Change Enable Register B
  DEFVALA = 0x06,  //!< Default Compare Register A
  DEFVALB = 0x07,  //!< Default Compare Register B
  INTCONA = 0x08,  //!< Interrupt Control Register A
  INTCONB = 0x09,  //!< Interrupt Control Register B
  IOCONA = 0x0A,   //!< Configuration Register A
  IOCONB = 0x0B,   //!< Configuration Register B
  GPPUA = 0x0C,    //!< Pull-Up Resistor Configuration Register A
  GPPUB = 0x0D,    //!< Pull-Up Resistor Configuration Register B
  INTFA = 0x0E,    //!< Interrupt Flag Register A
  INTFB = 0x0F,    //!< Interrupt Flag Register B
  INTCAPA = 0x10,  //!< Interrupt Captured Value Register A
  INTCAPB = 0x11,  //!< Interrupt Captured Value Register B
  GPIOA = 0x12,    //!< General Purpose I/O Register A
  GPIOB = 0x13,    //!< General Purpose I/O Register B
  OLATA = 0x14,    //!< Output Latch Register A
  OLATB = 0x15     //!< Output Latch Register B
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
enum class PIN_STATE { OFF, ON, UNDEFINED };
enum class GPIO_MODE {
  GPIO_INPUT = 0,
  GPIO_INPUT_PULLUP = 1,
  GPIO_OUTPUT = 2,
  NA = 3
};

enum class PULL_MODE { INTERNAL_PULLUP, PULL_DOWN, NONE };
enum class INTR_TYPE {
  INTR_CHANGE = 0,
  INTR_FALLING = 1,
  INTR_RISING = 2,
  NONE
};
enum class INTR_OUTPUT_TYPE {
  INTR_ACTIVE_HIGH = 0,
  INTR_ACTIVE_LOW = 1,
  INTR_OPEN_DRAIN = 2,
  NA = 3
};

} // namespace MCP
#endif