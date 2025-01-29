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
  std::array<uint8_t, MAX_REG_PER_PORT> registerAddress;
  std::array<uint8_t, MAX_REG_PER_PORT> registerSavedValues;
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
      : Pins(createPins(port)), pinInterruptState{false}, registerAddress{},
        registerSavedValues{}, model(m), bankMode(bankMerged),
        interruptEnabled(enableInterrupt),
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
      return nullptr;
    }
  }
  bool updateBankMode(bool value) {
    // icon register not need to be invoked again as this method is already
    // invoked and icon is updated  ,just notify this class for updating address
    bankMode = value;
    updateRegistersAddress();
    return true;
  }
  bool updateRegisterValue(uint8_t reg_address, uint8_t value) {
    MCP::REG regType = Util::findRegister(reg_address, bankMode);
    if (regType == REG::IOCON) {
      return iocon->updateState(value);
    } else {
      auto reg = getRegister(regType);
      return reg->updateState(value);
    }
  }

  // This method will not trigger any read event , only return register's
  // current saved value
  uint8_t getSavedValue(REG reg) const {
    return registerSavedValues[static_cast<uint8_t>(reg)];
  }
  uint8_t getAddress(REG reg) const {
    return registerAddress[static_cast<uint8_t>(reg)];
  }
  // Pin masks
  void setGeneralMask(MASK mask) { generalMask = static_cast<uint8_t>(mask); }
  void setGeneralMask(uint8_t mask) { generalMask = mask; }
  uint8_t getGeneralMask() const { return generalMask; }
  uint8_t getInterruptMask() const { return interruptMask; }

  // PIN DIRECTION SELECTION
  void setPinDirection(PIN p, GPIO_MODE m) {
    assert(Util::getPortFromPin(p) == port_name && "Invalid pin ");
    iodir->setPinMode<REG::IODIR>(p, m);
  }
  void setPinsAsOutput(uint8_t pinmask) {
    iodir->setPinMode<REG::IODIR>(pinmask, GPIO_MODE::GPIO_OUTPUT);
  }
  void setPinsAsInput(uint8_t pinmask) {
    iodir->setPinMode<REG::IODIR>(pinmask, GPIO_MODE::GPIO_INPUT);
  }
  void setPinsAsInput() {
    iodir->setPinMode<REG::IODIR>(generalMask, GPIO_MODE::GPIO_INPUT);
  }
  void setPinsAsOutput() {
    iodir->setPinMode<REG::IODIR>(generalMask, GPIO_MODE::GPIO_OUTPUT);
  }

  void setBankAsOutput() {
    uint8_t pinmask = static_cast<uint8_t>(MASK::ALL);
    iodir->setPinMode<REG::IODIR>(pinmask, GPIO_MODE::GPIO_OUTPUT);
  }

  void setBankAsInput() {
    uint8_t pinmask = static_cast<uint8_t>(MASK::ALL);
    iodir->setPinMode<REG::IODIR>(pinmask, GPIO_MODE::GPIO_INPUT);
  }
  // PULLUP SELECTION
  void setPullup(PIN pin, PULL_MODE mode) {
    gppu->setPullType<REG::GPPU>(pin, mode);
  }

  void setPinsPullup(uint8_t pinMask, PULL_MODE mode) {
    gppu->setPullType<REG::GPPU>(pinMask, mode);
  }

  void setPinsPullup(PULL_MODE mode) {
    gppu->setPullType<REG::GPPU>(generalMask, mode);
  }

  void setBankPullup(PULL_MODE mode) {
    uint8_t pinMask = static_cast<uint8_t>(MASK::ALL);
    gppu->setPullType<REG::GPPU>(pinMask, mode);
  }

  // Get PIN VALUE
  bool getPinState(MCP::PIN p) const {
    assert(Util::getPortFromPin(p) == port_name && "Invalid pin ");
    return gpio->readPin<REG::GPIO>(p);
  }
  uint8_t getPinState() const { return gpio->readPins<REG::GPIO>(); }
  bool getPinState(uint8_t pinmask) {
    return gpio->readPins<REG::GPIO>(pinmask);
  }
  // SET PIN POLARITY
  void setInputPolarity(PIN pin, INPUT_POLARITY pol) {
    assert(Util::getPortFromPin(pin) == port_name && "Invalid pin ");
    ipol->setInputPolarity<REG::IPOL>(pin, pol);
  }
  void setInputPolarity(uint8_t pinmask, INPUT_POLARITY pol) {
    ipol->setInputPolarity<REG::IPOL>(pinmask, pol);
  }
  void setInputPolarity(INPUT_POLARITY pol) {
    ipol->setInputPolarity<REG::IPOL>(generalMask, pol);
  }
  // SET PIN VALUE
  void setPinState(PIN pin, bool state) {
    assert(Util::getPortFromPin(pin) == port_name && "Invalid pin ");
    olat->setOutputLatch<REG::OLAT>(pin, state);
  }

  void setPinState(uint8_t pinmask, bool state) {
    olat->setOutputLatch<REG::OLAT>(pinmask, state);
  }

  void setPinState(bool state) {
    olat->setOutputLatch<REG::OLAT>(generalMask, state);
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
    setupRegisters();
    updateRegistersAddress();
    getSavedValues();
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
  void updateRegistersAddress() {
    if (iocon) {
      iocon->updatebankMode(bankMode);
      iocon->updateRegisterAddress(); // Ensure the address is updated if needed
      registerAddress[static_cast<size_t>(MCP::REG::IOCON)] =
          iocon->getAddress();
    }
    if (iodir) {
      iodir->updatebankMode(bankMode);
      iodir->updateRegisterAddress();
      registerAddress[static_cast<size_t>(MCP::REG::IODIR)] =
          iodir->getAddress();
    }
    if (gppu) {
      gppu->updatebankMode(bankMode);
      gppu->updateRegisterAddress();
      registerAddress[static_cast<size_t>(MCP::REG::GPPU)] = gppu->getAddress();
    }
    if (ipol) {
      ipol->updatebankMode(bankMode);
      ipol->updateRegisterAddress();
      registerAddress[static_cast<size_t>(MCP::REG::IPOL)] = ipol->getAddress();
    }
    if (gpio) {
      gpio->updatebankMode(bankMode);
      gpio->updateRegisterAddress();
      registerAddress[static_cast<size_t>(MCP::REG::GPIO)] = gpio->getAddress();
    }
    if (olat) {
      olat->updatebankMode(bankMode);
      olat->updateRegisterAddress();
      registerAddress[static_cast<size_t>(MCP::REG::OLAT)] = olat->getAddress();
    }
    if (gpinten) {
      gpinten->updatebankMode(bankMode);
      gpinten->updateRegisterAddress();
      registerAddress[static_cast<size_t>(MCP::REG::GPINTEN)] =
          gpinten->getAddress();
    }
    if (intcon) {
      intcon->updatebankMode(bankMode);
      intcon->updateRegisterAddress();
      registerAddress[static_cast<size_t>(MCP::REG::INTCON)] =
          intcon->getAddress();
    }
    if (defval) {
      defval->updatebankMode(bankMode);
      defval->updateRegisterAddress();
      registerAddress[static_cast<size_t>(MCP::REG::DEFVAL)] =
          defval->getAddress();
    }
    if (intf) {
      intf->updatebankMode(bankMode);
      intf->updateRegisterAddress();
      registerAddress[static_cast<size_t>(MCP::REG::INTF)] = intf->getAddress();
    }
    if (intcap) {
      intcap->updatebankMode(bankMode);
      intcap->updateRegisterAddress();
      registerAddress[static_cast<size_t>(MCP::REG::INTCAP)] =
          intcap->getAddress();
    }
  }
  void getSavedValues() {
    if (iocon) {

      registerSavedValues[static_cast<size_t>(MCP::REG::IOCON)] =
          iocon->getSavedValue();
    }
    if (iodir) {

      registerSavedValues[static_cast<size_t>(MCP::REG::IODIR)] =
          iodir->getSavedValue();
    }
    if (gppu) {

      registerSavedValues[static_cast<size_t>(MCP::REG::GPPU)] =
          gppu->getSavedValue();
    }
    if (ipol) {

      registerSavedValues[static_cast<size_t>(MCP::REG::IPOL)] =
          ipol->getSavedValue();
    }
    if (gpio) {

      registerSavedValues[static_cast<size_t>(MCP::REG::GPIO)] =
          gpio->getSavedValue();
    }
    if (olat) {

      registerSavedValues[static_cast<size_t>(MCP::REG::OLAT)] =
          olat->getSavedValue();
    }
    if (gpinten) {

      registerSavedValues[static_cast<size_t>(MCP::REG::GPINTEN)] =
          gpinten->getSavedValue();
    }
    if (intcon) {

      registerSavedValues[static_cast<size_t>(MCP::REG::INTCON)] =
          intcon->getSavedValue();
    }
    if (defval) {

      registerSavedValues[static_cast<size_t>(MCP::REG::DEFVAL)] =
          defval->getSavedValue();
    }
    if (intf) {

      registerSavedValues[static_cast<size_t>(MCP::REG::INTF)] =
          intf->getSavedValue();
    }
    if (intcap) {

      registerSavedValues[static_cast<size_t>(MCP::REG::INTCAP)] =
          intcap->getSavedValue();
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