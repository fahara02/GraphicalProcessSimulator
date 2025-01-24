#ifndef MCP_IO_HPP
#define MCP_IO_HPP

#include "MCP_Defines.hpp"
#include "MCP_Device.hpp"
#include "Wire.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <memory>
#include <variant>

#define MCP_TAG "MCP_IO"

namespace COMPONENT {

class MCP_IO_EXPANDER {

public:
  MCP_IO_EXPANDER(uint8_t address,
                  MCP::MCP_MODEL model = MCP::MCP_MODEL::MCP23017)
      : model_(model), address_(address), //
        sda_(GPIO_NUM_21),                //
        scl_(GPIO_NUM_22),                //
        cs_(GPIO_NUM_NC),                 //
        reset_(GPIO_NUM_33),              //
        wire_(std::make_unique<TwoWire>(0)) {

    switch (model) {
    case MCP::MCP_MODEL::MCP23017:
      device_ = createDevice<MCP::MCP_23X17::REG, MCP::MCP_MODEL::MCP23017>();
      break;
    case MCP::MCP_MODEL::MCP23S17:
      device_ = createDevice<MCP::MCP_23X17::REG, MCP::MCP_MODEL::MCP23S17>();
      break;
    case MCP::MCP_MODEL::MCP23018: // Example for an additional model
      device_ = createDevice<MCP::MCP_23X18::REG, MCP::MCP_MODEL::MCP23018>();
      break;
    case MCP::MCP_MODEL::MCP23S18: // Example for an additional model
      device_ = createDevice<MCP::MCP_23X18::REG, MCP::MCP_MODEL::MCP23S18>();
      break;
    default:
      ESP_LOGE(MCP_TAG, "Unsupported MCP model");
    }
  }

private:
  MCP::MCP_MODEL model_; // MCP model (e.g., MCP23017)
  uint8_t address_;
  gpio_num_t sda_;                // SDA GPIO pin
  gpio_num_t scl_;                // SCL GPIO pin
  gpio_num_t cs_;                 // Chip Select GPIO pin
  gpio_num_t reset_;              // Reset GPIO pin
  std::unique_ptr<TwoWire> wire_; // Unique pointer for TwoWire instance
  std::unique_ptr<MCP::MCPDeviceBase> device_;

  using regEnumMCP17 = MCP::MCPDevice<MCP::MCP_23X17::REG,
                                      MCP::MCP_MODEL::MCP23017>::RegEnumType;
  using regEnumMCP18 = MCP::MCPDevice<MCP::MCP_23X18::REG,
                                      MCP::MCP_MODEL::MCP23018>::RegEnumType;
  using RegEnum = std::variant<regEnumMCP17, regEnumMCP18>;

  struct regMap {
    RegEnum reg;
    uint8_t address;
  };
  std::array<regMap, MCP::MAX_REG_PER_PORT> registersPortA{};
  std::array<regMap, MCP::MAX_REG_PER_PORT> registersPortB{};

  void init();

  template <typename RegEnum, MCP::MCP_MODEL Model>
  std::unique_ptr<MCP::MCPDeviceBase> createDevice() {
    if constexpr (Model == MCP::MCP_MODEL::MCP23017) {
      return std::make_unique<
          MCP::MCPDevice<RegEnum, MCP::MCP_MODEL::MCP23017>>();
    } else if constexpr (Model == MCP::MCP_MODEL::MCP23S17) {
      return std::make_unique<
          MCP::MCPDevice<RegEnum, MCP::MCP_MODEL::MCP23S17>>();
    } else if constexpr (Model == MCP::MCP_MODEL::MCP23018) {
      return std::make_unique<
          MCP::MCPDevice<RegEnum, MCP::MCP_MODEL::MCP23018>>();
    } else if constexpr (Model == MCP::MCP_MODEL::MCP23S18) {
      return std::make_unique<
          MCP::MCPDevice<RegEnum, MCP::MCP_MODEL::MCP23S18>>();
    } else {
      ESP_LOGE(MCP_TAG, "Unknown MCP model");
      return nullptr;
    }
  }
  void setup(const MCP::register_icon_t &config) {
    if (device_) {
      device_->configure(config.getSettings());
    }
  }
  uint8_t getAddress(RegEnum reg, MCP::PORT port) const {

    const auto &registers =
        (port == MCP::PORT::GPIOA) ? registersPortA : registersPortB;

    for (size_t i = 0; i < registers.size(); ++i) {
      if (registers[i].reg == reg) {

        return registers[i].address;
      }
    }
    return 0xFF;
  }
  void updateAddressMap() {

    uint8_t *addressPortA = device_->generateAddress(MCP::PORT::GPIOA);
    uint8_t *addressPortB = device_->generateAddress(MCP::PORT::GPIOB);

    if (addressPortA != nullptr && addressPortB != nullptr) {
      for (size_t i = 0; i < MCP::MAX_REG_PER_PORT; ++i) {

        RegEnum reg = (model_ == MCP::MCP_MODEL::MCP23017 ||
                       model_ == MCP::MCP_MODEL::MCP23S17)
                          ? RegEnum(static_cast<regEnumMCP17>(i))
                          : RegEnum(static_cast<regEnumMCP18>(i));

        // Populate the registers for PORTA and PORTB
        registersPortA[i] = {reg, addressPortA[i]};
        registersPortB[i] = {reg, addressPortB[i]};
      }
    }
  }
};

} // namespace COMPONENT

#endif // MCP23017_HPP
