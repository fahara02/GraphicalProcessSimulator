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
  // static constexpr uint8_t getIndexFromPin(PIN pin) {
  //   uint8_t index = 0;
  //   PORT port = getPortFromPin(pin);
  //   uint8_t pinEnum = static_cast<uint8_t>(pin);
  //   return index = (port == PORT::GPIOB) ? (pinEnum - 8) : pinEnum;
  // }
  static constexpr uint8_t calculateAddress(REG reg, PORT port, bool bankMode) {
    uint8_t baseAddress = static_cast<uint8_t>(reg);

    if (bankMode) {
      // BANK = 1: Separate PORTA and PORTB
      return baseAddress + (port == PORT::GPIOB ? 0x10 : 0x00);
    } else {
      // BANK = 0: Paired A/B registers
      return (baseAddress * 2) + (port == PORT::GPIOB ? 0x01 : 0x00);
    }

    return baseAddress;
  }
  static constexpr MCP::REG findRegister(uint8_t address, bool bankMode) {
    if (bankMode) {
      // BANK = 1: Separate PORTA and PORTB
      return static_cast<MCP::REG>(
          address & 0xEF); // Mask out the PORT offset (0x10 for GPIOB)
    } else {
      // BANK = 0: Paired A/B registers
      return static_cast<MCP::REG>(address / 2);
    }
  }
  class BIT {
  public:
    static void set(uint8_t &byte, uint8_t bit) { byte |= (1UL << bit); }
    static void clear(uint8_t &byte, uint8_t bit) { byte &= ~(1UL << bit); }
    static bool isSet(const uint8_t &byte, uint8_t bit) {
      return (byte & (1UL << bit)) != 0U;
    }
    static bool isAllSet(const uint8_t &byte, uint8_t mask) {
      return (byte & mask) == mask;
    }
    static bool isAnySet(const uint8_t &byte, uint8_t mask) {
      return (byte & mask) != 0;
    }

    static bool isSingleBit(uint8_t mask) {
      return (mask != 0) && ((mask & (mask - 1)) == 0);
    }
  };

  class ToString {
  public:
    static const char *REG(MCP::REG reg) {
      switch (reg) {
      case MCP::REG::IODIR:
        return "IODIR";
      case MCP::REG::IPOL:
        return "IPOL";
      case MCP::REG::GPINTEN:
        return "GPINTEN";
      case MCP::REG::DEFVAL:
        return "DEFVAL";
      case MCP::REG::INTCON:
        return "INTCON";
      case MCP::REG::IOCON:
        return "IOCON";
      case MCP::REG::GPPU:
        return "GPPU";
      case MCP::REG::INTF:
        return "INTF";
      case MCP::REG::INTCAP:
        return "INTCAP";
      case MCP::REG::GPIO:
        return "GPIO";
      case MCP::REG::OLAT:
        return "OLAT";
      default:
        return "UNKNOWN";
      }
    }
  };
};

} // namespace MCP

#endif