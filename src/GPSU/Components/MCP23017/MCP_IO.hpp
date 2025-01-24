#ifndef MCP_IO_HPP
#define MCP_IO_HPP

#include "MCP_Defines.hpp"
#include "MCP_Device.hpp"
#include "Wire.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <array>
#include <memory>
#include <variant>

#define MCP_TAG "MCP_IO"

namespace COMPONENT {

class MCP_IO_EXPANDER {

public:
  template <typename MCPChip>
  MCP_IO_EXPANDER(uint8_t address, MCPChip &mcpChip)
      : address_(address),                                        //
        sda_(GPIO_NUM_21),                                        //
        scl_(GPIO_NUM_22),                                        //
        cs_(GPIO_NUM_NC),                                         //
        reset_(GPIO_NUM_33),                                      //
        wire_(std::make_unique<TwoWire>(0)),                      //
        device_(std::make_unique<typename MCPChip::DeviceType>()) //

  {
    model_ = mcpChip.getModel();
    setupDevice(mcpChip);
  }
  void printRegisters() const {
    printf("Printing Registers for MCP_IO_EXPANDER (Address: 0x%02X):\n",
           address_);

    printf("PORTA Registers:\n");
    for (const auto &entry : registersPortA) {
      printf("Register 0x%02X: 0x%02X\n", entry.first, entry.second);
    }

    printf("PORTB Registers:\n");
    for (const auto &entry : registersPortB) {
      printf("Register 0x%02X: 0x%02X\n", entry.first, entry.second);
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

  std::array<std::pair<int, uint8_t>, MCP::MAX_REG_PER_PORT> registersPortA{};
  std::array<std::pair<int, uint8_t>, MCP::MAX_REG_PER_PORT> registersPortB{};
  void init();
  template <typename MCPChip> void setupDevice(const MCPChip &mcpStruct) {

    updateAddressMap<MCPChip>();
  }

  void configure(const MCP::register_icon_t &config) {
    if (device_) {
      device_->configure(config.getSettings());
    }
  }
  template <typename MCPChip>
  uint8_t getAddress(typename MCPChip::RegEnumType reg, MCP::PORT port) const {
    const auto &registers =
        (port == MCP::PORT::GPIOA) ? registersPortA : registersPortB;

    for (const auto &entry : registers) {
      if (entry.first == static_cast<int>(reg)) {
        return entry.second;
      }
    }
    return 0xFF;
  }
  template <typename MCPChip> void updateAddressMap() {
    uint8_t *addressPortA = device_->generateAddress(MCP::PORT::GPIOA);
    uint8_t *addressPortB = device_->generateAddress(MCP::PORT::GPIOB);

    if (addressPortA != nullptr && addressPortB != nullptr) {
      for (size_t i = 0; i < MCP::MAX_REG_PER_PORT; ++i) {
        typename MCPChip::RegEnumType reg =
            static_cast<typename MCPChip::RegEnumType>(i);

        // Populate the registers for PORTA and PORTB
        registersPortA[i] = {static_cast<int>(reg), addressPortA[i]};
        registersPortB[i] = {static_cast<int>(reg), addressPortB[i]};
      }
    }
  }
};

} // namespace COMPONENT

#endif // MCP23017_HPP
