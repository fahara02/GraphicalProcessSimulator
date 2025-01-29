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
  std::unordered_map<MCP::REG, std::unique_ptr<MCP::MCPRegister>> regMap;
  std::unordered_map<uint8_t, MCP::REG> addressMap;
  // Control Register
  std::shared_ptr<MCP::MCPRegister> iocon;

public:
  GPIO_BANK(PORT port, MCP::MCP_MODEL m = MCP::MCP_MODEL::MCP23017,
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
  MCPRegister *getRegister(MCP::REG regType) {
    if (regType == REG::IOCON)
      return iocon.get();
    auto it = regMap.find(regType);
    return (it != regMap.end()) ? it->second.get() : nullptr;
  }

  uint8_t getSavedValue(REG reg) const {
    if (reg == REG::IOCON) {
      return iocon->getSavedValue();
    } else {
      auto it = regMap.find(reg);
      return it != regMap.end() ? it->second->getSavedValue() : 0;
    }
  }

  uint8_t getAddress(REG reg) const {
    if (reg == REG::IOCON) {
      return iocon->getAddress();
    } else {
      auto it = regMap.find(reg);
      return it != regMap.end() ? it->second->getAddress() : 0;
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
    if (iocon->getAddress() == reg_address) {
      return iocon->updateState(value);
    }

    for (auto &[regType, regPtr] : regMap) {
      if (regPtr->getAddress() == reg_address) {
        return regPtr->updateState(value);
      }
    }

    ESP_LOGE(BANK_TAG, "Failed to update register: Invalid address (0x%02X)",
             reg_address);
    return false;
  }

  // This method will not trigger any read event , only return register's
  // current saved value

  // Pin masks
  void setGeneralMask(MASK mask) { generalMask = static_cast<uint8_t>(mask); }
  void setGeneralMask(uint8_t mask) { generalMask = mask; }
  uint8_t getGeneralMask() const { return generalMask; }
  uint8_t getInterruptMask() const { return interruptMask; }

  // PIN DIRECTION SELECTION
  void setPinDirection(PIN p, GPIO_MODE m) {
    assert(Util::getPortFromPin(p) == port_name && "Invalid pin ");
    getRegister(MCP::REG::IODIR)->setPinMode<REG::IODIR>(p, m);
  }
  void setPinsAsOutput(uint8_t pinmask) {
    getRegister(MCP::REG::IODIR)
        ->setPinMode<REG::IODIR>(pinmask, GPIO_MODE::GPIO_OUTPUT);
  }
  void setPinsAsInput(uint8_t pinmask) {
    getRegister(MCP::REG::IODIR)
        ->setPinMode<REG::IODIR>(pinmask, GPIO_MODE::GPIO_INPUT);
  }
  void setPinsAsInput() {
    getRegister(MCP::REG::IODIR)
        ->setPinMode<REG::IODIR>(generalMask, GPIO_MODE::GPIO_INPUT);
  }
  void setPinsAsOutput() {
    getRegister(MCP::REG::IODIR)
        ->setPinMode<REG::IODIR>(generalMask, GPIO_MODE::GPIO_OUTPUT);
  }

  void setBankAsOutput() {
    uint8_t pinmask = static_cast<uint8_t>(MASK::ALL);
    getRegister(MCP::REG::IODIR)
        ->setPinMode<REG::IODIR>(pinmask, GPIO_MODE::GPIO_OUTPUT);
  }

  void setBankAsInput() {
    uint8_t pinmask = static_cast<uint8_t>(MASK::ALL);
    getRegister(MCP::REG::IODIR)
        ->setPinMode<REG::IODIR>(pinmask, GPIO_MODE::GPIO_INPUT);
  }
  // PULLUP SELECTION
  void setPullup(PIN pin, PULL_MODE mode) {
    getRegister(MCP::REG::GPPU)->setPullType<REG::GPPU>(pin, mode);
  }

  void setPinsPullup(uint8_t pinMask, PULL_MODE mode) {
    getRegister(MCP::REG::GPPU)->setPullType<REG::GPPU>(pinMask, mode);
  }

  void setPinsPullup(PULL_MODE mode) {
    getRegister(MCP::REG::GPPU)->setPullType<REG::GPPU>(generalMask, mode);
  }

  void setBankPullup(PULL_MODE mode) {
    uint8_t pinMask = static_cast<uint8_t>(MASK::ALL);
    getRegister(MCP::REG::GPPU)->setPullType<REG::GPPU>(pinMask, mode);
  }

  // Get PIN VALUE
  bool getPinState(MCP::PIN p) {
    assert(Util::getPortFromPin(p) == port_name && "Invalid pin ");
    return getRegister(MCP::REG::GPIO)->readPin<REG::GPIO>(p);
  }
  uint8_t getPinState() {
    return getRegister(MCP::REG::GPIO)->readPins<REG::GPIO>();
  }
  bool getPinState(uint8_t pinmask) {
    return getRegister(MCP::REG::GPIO)->readPins<REG::GPIO>(pinmask);
  }
  // SET PIN POLARITY
  void setInputPolarity(PIN pin, INPUT_POLARITY pol) {
    assert(Util::getPortFromPin(pin) == port_name && "Invalid pin ");
    getRegister(MCP::REG::IPOL)->setInputPolarity<REG::IPOL>(pin, pol);
  }
  void setInputPolarity(uint8_t pinmask, INPUT_POLARITY pol) {
    getRegister(MCP::REG::IPOL)->setInputPolarity<REG::IPOL>(pinmask, pol);
  }
  void setInputPolarity(INPUT_POLARITY pol) {
    getRegister(MCP::REG::IPOL)->setInputPolarity<REG::IPOL>(generalMask, pol);
  }
  // SET PIN VALUE
  void setPinState(PIN pin, bool state) {
    assert(Util::getPortFromPin(pin) == port_name && "Invalid pin ");
    getRegister(MCP::REG::OLAT)->setOutputLatch<REG::OLAT>(pin, state);
  }

  void setPinState(uint8_t pinmask, bool state) {
    getRegister(MCP::REG::OLAT)->setOutputLatch<REG::OLAT>(pinmask, state);
  }

  void setPinState(bool state) {
    getRegister(MCP::REG::OLAT)->setOutputLatch<REG::OLAT>(generalMask, state);
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

    updatePins();
    if (interruptEnabled) {
      updatePinInterruptState();
    }
  }
  void setupRegisters() {
    iocon = std::make_shared<MCP::MCPRegister>(model, MCP::REG::IOCON,
                                               port_name, bankMode);

    for (auto regType :
         {MCP::REG::IODIR, MCP::REG::GPPU, MCP::REG::IPOL, MCP::REG::GPIO,
          MCP::REG::OLAT, MCP::REG::GPINTEN, MCP::REG::INTCON, MCP::REG::DEFVAL,
          MCP::REG::INTF, MCP::REG::INTCAP}) {
      auto reg = std::make_unique<MCP::MCPRegister>(model, regType, port_name,
                                                    bankMode);
      regMap[regType] = std::move(reg);
    }

    updateAddressMap(); // Ensure addressMap is initialized separately
  }

  void updateRegistersAddress() {
    if (iocon) {
      iocon->updatebankMode(bankMode);
      iocon->updateRegisterAddress();
    }

    for (auto &[regType, regPtr] : regMap) {
      regPtr->updatebankMode(bankMode);
      regPtr->updateRegisterAddress();
    }

    updateAddressMap(); // Update addressMap when addresses change
  }
  void updateAddressMap() {
    addressMap.clear();
    if (iocon) {
      addressMap[iocon->getAddress()] = MCP::REG::IOCON;
    }

    for (const auto &[regType, regPtr] : regMap) {
      addressMap[regPtr->getAddress()] = regType;
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