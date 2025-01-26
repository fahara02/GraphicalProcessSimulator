#ifndef MCP_GPIO_BANKS_HPP
#define MCP_GPIO_BANKS_HPP
#include "MCP_Primitives.hpp"
#include "MCP_Registers.hpp"
#include "esp_log.h"
#include "memory"

#define BANK_TAG "GPIO_BANK"
namespace MCP {
class GPIO_BANK {
private:
  std::array<Pin, PIN_PER_BANK> Pins;
  std::array<bool, PIN_PER_BANK> pinInterruptState;

  // Control Register
  std::shared_ptr<MCP::MCPRegister> iocon;
  // Basic GPIO Read & Write
  std::unique_ptr<MCP::MCPRegister> iodir;
  std::unique_ptr<MCP::MCPRegister> gppu;
  std::unique_ptr<MCP::MCPRegister> ipol;
  std::unique_ptr<MCP::MCPRegister> gpio;
  std::unique_ptr<MCP::MCPRegister> olat;
  // Interrupt
  std::unique_ptr<MCP::MCPRegister> gpinten;
  std::unique_ptr<MCP::MCPRegister> intcon;
  std::unique_ptr<MCP::MCPRegister> defval;
  std::unique_ptr<MCP::MCPRegister> intf;
  std::unique_ptr<MCP::MCPRegister> intcap;

public:
  constexpr GPIO_BANK(PORT port, MCP::MCP_MODEL m = MCP::MCP_MODEL::MCP23017,
                      bool bankMerged = false, bool enableInterrupt = false)
      : Pins(createPins(port)), pinInterruptState{false}, model(m),
        bankMode(bankMerged), interruptEnabled(enableInterrupt),
        generalMask(static_cast<uint8_t>(MASK::ALL)), interruptMask(0x00),
        port_name(port), intr_type(INTR_TYPE::NONE),
        intr_out_type(INTR_OUTPUT_TYPE::NA) {
    assert(isValidPort(port) && "Invalid PORT provided!");
    init();
  }
  std::shared_ptr<MCP::MCPRegister> const getControlRegister() { return iocon; }
  MCPRegister *const getRegister(MCP::REG regType) {
    switch (regType) {
    case MCP::REG::IOCON:
      return nullptr;
    case MCP::REG::IODIR:
      return iodir.get();
    case MCP::REG::GPPU:
      return gppu.get();
    case MCP::REG::IPOL:
      return ipol.get();
    case MCP::REG::GPIO:
      return gpio.get();
    case MCP::REG::OLAT:
      return olat.get();
    case MCP::REG::GPINTEN:
      return gpinten.get();
    case MCP::REG::INTCON:
      return intcon.get();
    case MCP::REG::DEFVAL:
      return defval.get();
    case MCP::REG::INTF:
      return intf.get();
    case MCP::REG::INTCAP:
      return intcap.get();
    default:
      ESP_LOGE(BANK_TAG, "Invalid register type requested!");
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
  bool getPinState(MCP::PIN p) const { return gpio->readPin<REG::GPIO>(p); }
  uint8_t getPinStates() const { return gpio->readPins<REG::GPIO>(); }

  void setPinState(PIN pin, bool state) {

    assert(Util::getPortFromPin(pin) == port_name && "Invalid pin ");

    olat->setOutputLatch<REG::OLAT>(pin, state);
  }

  void setPinState(bool state) {
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
  MCP::MCP_MODEL model;
  bool bankMode;
  bool interruptEnabled;
  uint8_t generalMask;
  uint8_t interruptMask;
  PORT port_name;
  INTR_TYPE intr_type;
  INTR_OUTPUT_TYPE intr_out_type;
  void init() {
    setupRegisters();
    updatePins();
    if (interruptEnabled) {
      updatePinInterruptState();
    }
  }
  void setupRegisters() {

    iocon = std::make_shared<MCP::MCPRegister>(model, MCP::REG::IOCON,
                                               port_name, bankMode);
    iodir = std::make_unique<MCP::MCPRegister>(model, MCP::REG::IODIR,
                                               port_name, bankMode);
    gppu = std::make_unique<MCP::MCPRegister>(model, MCP::REG::GPPU, port_name,
                                              bankMode);
    ipol = std::make_unique<MCP::MCPRegister>(model, MCP::REG::IPOL, port_name,
                                              bankMode);
    gpio = std::make_unique<MCP::MCPRegister>(model, MCP::REG::GPIO, port_name,
                                              bankMode);
    olat = std::make_unique<MCP::MCPRegister>(model, MCP::REG::OLAT, port_name,
                                              bankMode);
    gpinten = std::make_unique<MCP::MCPRegister>(model, MCP::REG::GPINTEN,
                                                 port_name, bankMode);
    intcon = std::make_unique<MCP::MCPRegister>(model, MCP::REG::INTCON,
                                                port_name, bankMode);
    defval = std::make_unique<MCP::MCPRegister>(model, MCP::REG::DEFVAL,
                                                port_name, bankMode);
    intf = std::make_unique<MCP::MCPRegister>(model, MCP::REG::INTF, port_name,
                                              bankMode);
    intcap = std::make_unique<MCP::MCPRegister>(model, MCP::REG::INTCAP,
                                                port_name, bankMode);
  }
  void setInterruptMask(uint8_t pinMask) {
    interruptMask = pinMask & 0xFF;
    if (interruptEnabled) {
      updatePinInterruptState();
    }
  }
  void updatePins() {
    for (uint8_t i = 0; i < PIN_PER_BANK; ++i) {
      bool isGeneralMaskSet = Util::BIT::isSet(generalMask, i);

      if (!isGeneralMaskSet) {

        Pins[i].setState(false);
      }
    }
  }

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
// constexpr GPIO_BANKS<MCP23017Pin> gpioBankA(PORT::GPIOA);
// // Define GPIO Bank instances
// GPIO_BANKS BANK_A(PORT::GPIOA);

// // Using the variadic constructor
// constexpr GPIO_BANKS bankA_custom(PORT::GPIOA, GPA0, GPA1, GPA2);
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