#ifndef MCP_DEVICE_HPP
#define MCP_DEVICE_HPP

#include "MCP_Primitives.hpp"
#include "MCP_Registers.hpp"
#include "Wire.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <array>
#include <memory>
#include <variant>

#define MCP_TAG "MCP_IO"

namespace COMPONENT {
template <typename MCPChip> //
class MCPDevice {
private:
  MCP::MCP_MODEL model_; // MCP model (e.g., MCP23017)
  uint8_t address_;
  gpio_num_t sda_;                // SDA GPIO pin
  gpio_num_t scl_;                // SCL GPIO pin
  gpio_num_t cs_;                 // Chip Select GPIO pin
  gpio_num_t reset_;              // Reset GPIO pin
  std::unique_ptr<TwoWire> wire_; // Unique pointer for TwoWire instance

  bool bankMode_;

public:
  std::unique_ptr<MCP::ioconBase> controlRegister;
  using RegisterType = typename MCPChip::RegisterType;
  using RegEnumType = typename MCPChip::RegEnumType;

  MCPDevice(uint8_t address, MCPChip &mcpChip)
      : address_(address),                   //
        sda_(GPIO_NUM_21),                   //
        scl_(GPIO_NUM_22),                   //
        cs_(GPIO_NUM_NC),                    //
        reset_(GPIO_NUM_33),                 //
        wire_(std::make_unique<TwoWire>(0)), //
        controlRegister(std::make_unique<typename MCPChip::ICONType>())
  //

  {
    controlRegister->setCallBack(
        [this]() { MCPDevice::updateRegisters(this); });
    model_ = mcpChip.getModel();
    bankMode_ = controlRegister->getBankMode();
    initializeRegisters<MCPChip>(MCP::PORT::GPIOA, registersPortA, bankMode_);
    initializeRegisters<MCPChip>(MCP::PORT::GPIOB, registersPortB, bankMode_);
  }

  RegisterType *getRegister(RegEnumType reg, MCP::PORT port) {
    auto &registers =
        (port == MCP::PORT::GPIOA) ? registersPortA : registersPortB;

    size_t index = static_cast<size_t>(reg);
    if (index >= MCP::MAX_REG_PER_PORT) {
      ESP_LOGE(MCP_TAG, "Invalid register index: %d", index);
      return nullptr;
    }

    return registers[index].get();
  }
  void dumpRegisters() const {

    ESP_LOGI(MCP_TAG, "Dumping Registers for MCP_Device (Address: 0x%02X)",
             address_);

    // Dump PORTA Registers
    ESP_LOGI(MCP_TAG, "PORTA Registers:");
    for (size_t i = 0; i < MCP::MAX_REG_PER_PORT; ++i) {
      const auto &reg = registersPortA[i];
      ESP_LOGI(MCP_TAG, "Index: %d, Address: 0x%02X, Value: 0x%02X", i,
               reg->getAddress(), reg->getValue());
    }

    // Dump PORTB Registers
    ESP_LOGI(MCP_TAG, "PORTB Registers:");
    for (size_t i = 0; i < MCP::MAX_REG_PER_PORT; ++i) {
      const auto &reg = registersPortB[i];
      ESP_LOGI(MCP_TAG, "Index: %d, Address: 0x%02X, Value: 0x%02X", i,
               reg->getAddress(), reg->getValue());
    }
  }

private:
  std::array<std::unique_ptr<RegisterType>, MCP::MAX_REG_PER_PORT>
      registersPortA;
  std::array<std::unique_ptr<RegisterType>, MCP::MAX_REG_PER_PORT>
      registersPortB;

  void init();
  void setupDevice(const MCPChip &mcpStruct);

  void configure(const MCP::config_icon_t &config) {
    if (controlRegister) {
      controlRegister->configure(config.getSettings());
    }
  }
  static void updateRegisters(MCPDevice *device) {
    if (device) {
      for (size_t i = 0; i < MCP::MAX_REG_PER_PORT; ++i) {
        device->registersPortA[i]->updateRegisterAddress();
        device->registersPortB[i]->updateRegisterAddress();
      }
    }
  }

  template <typename Chip>
  void initializeRegisters(MCP::PORT port, //
                           std::array<std::unique_ptr<RegisterType>,
                                      MCP::MAX_REG_PER_PORT> &registers,
                           bool bankMode) {

    for (size_t i = 0; i < MCP::MAX_REG_PER_PORT; ++i) {
      auto regEnum = static_cast<RegEnumType>(i);
      registers[i] = std::make_unique<RegisterType>(regEnum, port, bankMode);
      registers[i]->updateRegisterAddress();
    }
  }
};

} // namespace COMPONENT

#endif
