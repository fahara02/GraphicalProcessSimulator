#ifndef MCP_GPIO_BANKS_HPP
#define MCP_GPIO_BANKS_HPP
#include "MCP_Primitives.hpp"
#include "MCP_Registers.hpp"
#include "esp_log.h"
#include "memory"
#include <unordered_map>
#define BANK_TAG "GPIO_BANK"
namespace MCP {

class GPIO_BANK {
private:
  std::array<Pin, PIN_PER_BANK> Pins;
  std::array<bool, PIN_PER_BANK> pinInterruptState;
  MCPRegisters regs;

public:
  GPIO_BANK(PORT port, MCP::MCP_MODEL m = MCP::MCP_MODEL::MCP23017,
            bool bankMerged = false, bool enableInterrupt = false)
      : Pins(createPins(port)), pinInterruptState{false}, model(m),
        bankMode(bankMerged), interruptEnabled(enableInterrupt),
        generalMask(static_cast<uint8_t>(MASK::ALL)), interruptMask(0x00),
        port_name(port), intr_type(INTR_TYPE::NONE),
        intr_out_type(INTR_OUTPUT_TYPE::NA) {
    assert(isValidPort(port) && "Invalid PORT provided!");

    regs.setup(model, port_name, bankMode);
    init();
  }

  std::shared_ptr<MCP::MCPRegister> const getControlRegister() {
    return regs.getIOCON();
  }
  const MCPRegister *getRegister(MCP::REG regType) const {
    return regs.getRegister(regType);
  }
  MCPRegister *getRegisterForUpdate(MCP::REG regType) {
    return regs.getRegisterForUpdate(regType);
  }

  uint8_t getAddress(REG reg) const { return regs.getAddress(reg); }

  bool updateBankMode(bool value) {
    // icon register not need to be invoked again as this method is already
    // invoked and icon is updated  ,just notify this class for updating address
    bankMode = value;
    regs.updateAddress(bankMode);
    return true;
  }
  bool updateRegisterValue(uint8_t reg_address, uint8_t value) {
    return regs.updateRegisterValue(reg_address, value);
  }

  // This method will not trigger any read event , only return register's
  // current saved value
  uint8_t getSavedValue(REG reg) const { return regs.getSavedValue(reg); }

  // Pin masks
  void setGeneralMask(MASK mask) { generalMask = static_cast<uint8_t>(mask); }
  void setGeneralMask(uint8_t mask) { generalMask = mask; }
  uint8_t getGeneralMask() const { return generalMask; }
  uint8_t getInterruptMask() const { return interruptMask; }

  // PIN DIRECTION SELECTION
  void setPinDirection(PIN p, GPIO_MODE m) {
    assert(Util::getPortFromPin(p) == port_name && "Invalid pin ");
    regs.iodir->setPinMode<REG::IODIR>(p, m);
  }
  void setPinDirection(uint8_t pinmask, GPIO_MODE m) {
    regs.iodir->setPinMode<REG::IODIR>(pinmask, m);
  }

  void setPinDirection(GPIO_MODE m) {
    uint8_t pinmask = static_cast<uint8_t>(MASK::ALL);
    regs.iodir->setPinMode<REG::IODIR>(pinmask, m);
  }

  // PULLUP SELECTION
  void setPullup(PIN pin, PULL_MODE mode) {
    regs.gppu->setPullType<REG::GPPU>(pin, mode);
  }

  void setPullup(uint8_t pinMask, PULL_MODE mode) {
    regs.gppu->setPullType<REG::GPPU>(pinMask, mode);
  }

  void setPullup(PULL_MODE mode) {
    regs.gppu->setPullType<REG::GPPU>(generalMask, mode);
  }

  void setBankPullup(PULL_MODE mode) {
    uint8_t pinMask = static_cast<uint8_t>(MASK::ALL);
    regs.gppu->setPullType<REG::GPPU>(pinMask, mode);
  }

  // Get PIN VALUE
  bool getPinState(MCP::PIN p) {
    assert(Util::getPortFromPin(p) == port_name && "Invalid pin ");
    return regs.gpio->readPin<REG::GPIO>(p);
  }
  uint8_t getPinState() { return regs.gpio->readPins<REG::GPIO>(); }
  bool getPinState(uint8_t pinmask) {
    return regs.gpio->readPins<REG::GPIO>(pinmask);
  }
  // SET PIN POLARITY
  void setInputPolarity(PIN pin, INPUT_POLARITY pol) {
    assert(Util::getPortFromPin(pin) == port_name && "Invalid pin ");
    regs.ipol->setInputPolarity<REG::IPOL>(pin, pol);
  }
  void setInputPolarity(uint8_t pinmask, INPUT_POLARITY pol) {
    regs.ipol->setInputPolarity<REG::IPOL>(pinmask, pol);
  }
  void setInputPolarity(INPUT_POLARITY pol) {
    regs.ipol->setInputPolarity<REG::IPOL>(generalMask, pol);
  }
  // SET PIN VALUE
  void setPinState(PIN pin, bool state) {
    assert(Util::getPortFromPin(pin) == port_name && "Invalid pin ");
    regs.olat->setOutputLatch<REG::OLAT>(pin, state);
  }

  void setPinState(uint8_t pinmask, bool state) {
    regs.olat->setOutputLatch<REG::OLAT>(pinmask, state);
  }

  void setPinState(bool state) {
    regs.olat->setOutputLatch<REG::OLAT>(generalMask, state);
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

  bool isInterruptEnabled() const { return interruptEnabled; }

private:
  MCP::MCP_MODEL model;
  bool bankMode;
  bool interruptEnabled;
  uint8_t generalMask;
  uint8_t interruptMask;
  PORT port_name;
  INTR_TYPE intr_type;
  INTR_OUTPUT_TYPE intr_out_type;
  void init() {
    updatePins();
    if (interruptEnabled) {
      updatePinInterruptState();
    }
  }

  void setInterruptMask(uint8_t pinMask) {
    interruptMask = pinMask & 0xFF;
    if (interruptEnabled) {
      updatePinInterruptState();
    }
  }
  void updatePins() {}

  void updatePinInterruptState() {
    for (uint8_t i = 0; i < PIN_PER_BANK; ++i) {
      bool isGeneralMaskSet = Util::BIT::isSet(generalMask, i);
      bool isInterruptMaskSet = Util::BIT::isSet(interruptMask, i);
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
};

} // namespace MCP
#endif

//  template <typename... PinType>
//   constexpr GPIO_BANKS(PORT port, PinType... pins)
//       : Pins(fillPinsArray(pins...)), pinInterruptState{false},
//         interruptEnabled(false), generalMask(calculateMask(pins...)),
//         interruptMask(0x00), port_name(port), ports(0), ddr(0), pull_ups(0),
//         intr_type(INTR_TYPE::NONE), intr_out_type(INTR_OUTPUT_TYPE::NA) {
//     // Validate port
//     assert(isValidPort(port) && "Invalid PORT provided!");

//     // Validate that all pins belong to the same port
//     assert(validatePins(port, pins...) &&
//            "All pins must belong to the same port!");
//   }

// // Helper to calculate the general mask
// template <typename... PinType>
// static constexpr uint8_t calculateMask(PinType... pins) {
//   return (0 | ... | pins.getMask());
// }

// // Helper to validate pins belong to the same port
// template <typename... PinType>
// static constexpr bool validatePins(PORT port, PinType... pins) {
//   return ((pins.getPort() == port) && ...);
// }
// template <typename... PinType>
// static constexpr std::array<Pin, PIN_PER_BANK> fillPinsArray(PinType... pins)
// {
//   std::array<Pin, PIN_PER_BANK> pinArray = {};
//   size_t i = 0;

//   // Assign provided pins
//   ((pinArray[i++] = pins), ...);

//   // Fill remaining Pins with default
//   while (i < PIN_PER_BANK) {
//     pinArray[i++] = Pin();
//   }

//   return pinArray;
// }