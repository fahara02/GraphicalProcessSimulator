#ifndef MCP_DEVICE_HPP
#define MCP_DEVICE_HPP

#include "MCP_GPIO_banks.hpp"
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

class MCPDevice {
private:
  MCP::MCP_MODEL model_;
  uint8_t address_;
  gpio_num_t sda_;
  gpio_num_t scl_;
  gpio_num_t cs_;
  gpio_num_t reset_;
  std::unique_ptr<TwoWire> wire_;
  bool bankMode_;

public:
  std::unique_ptr<MCP::GPIO_BANK> gpioBankA;
  std::unique_ptr<MCP::GPIO_BANK> gpioBankB;
  std::shared_ptr<MCP::MCPRegister> cntrlRegA;
  std::shared_ptr<MCP::MCPRegister> cntrlRegB;

  MCPDevice(uint8_t address, MCP::MCP_MODEL model)
      : model_(model), address_(address),    //
        sda_(GPIO_NUM_21),                   //
        scl_(GPIO_NUM_22),                   //
        cs_(GPIO_NUM_NC),                    //
        reset_(GPIO_NUM_33),                 //
        wire_(std::make_unique<TwoWire>(0)), //
        gpioBankA(std::make_unique<MCP::GPIO_BANK>(MCP::PORT::GPIOA, model)),
        gpioBankB(std::make_unique<MCP::GPIO_BANK>(MCP::PORT::GPIOB, model)),
        cntrlRegA(gpioBankA->getControlRegister()),
        cntrlRegB(gpioBankB->getControlRegister())

  {

    bankMode_ = cntrlRegA->getBankMode<MCP::REG::IOCON>();
  }

  void configure(const MCP::config_icon_t &config) {
    if (cntrlRegA && cntrlRegB) {
      cntrlRegA->configure<MCP::REG::IOCON>(config.getSettings());
      cntrlRegB->configure<MCP::REG::IOCON>(config.getSettings());
    }
  }
  MCP::MCPRegister *getRegister(MCP::REG reg, MCP::PORT port) {
    if (port == MCP::PORT::GPIOA) {

      return gpioBankA->getRegister(reg);

    } else {

      return gpioBankB->getRegister(reg);
    }
  }
  static void updateRegisters(MCPDevice *device) {
    if (device) {
    }
  }

private:
  void init();
  void setupDevice();
  uint8_t read_mcp_register(const uint8_t reg) {
    wire_->beginTransmission(address_);
    wire_->write(reg);
    wire_->endTransmission(false);
    wire_->requestFrom((uint8_t)address_, (uint8_t)1, (uint8_t) true);
    while (wire_->available() == 0)
      ;

    return wire_->read();
  }
  void write_mcp_register(const uint8_t reg, uint8_t value) {
    wire_->beginTransmission(address_);
    wire_->write(reg);
    wire_->write(value);
    wire_->endTransmission(true);
  }
};

} // namespace COMPONENT

#endif

// void dumpRegisters() const {

//   ESP_LOGI(MCP_TAG, "Dumping Registers for MCP_Device (Address: 0x%02X)",
//            address_);

//   // Dump PORTA Registers
//   ESP_LOGI(MCP_TAG, "PORTA Registers:");
//   for (size_t i = 0; i < MCP::MAX_REG_PER_PORT; ++i) {
//     const auto &reg = registersPortA[i];
//     ESP_LOGI(MCP_TAG, "Index: %d, Address: 0x%02X, Value: 0x%02X", i,
//              reg->getAddress(), reg->getValue());
//   }

//   // Dump PORTB Registers
//   ESP_LOGI(MCP_TAG, "PORTB Registers:");
//   for (size_t i = 0; i < MCP::MAX_REG_PER_PORT; ++i) {
//     const auto &reg = registersPortB[i];
//     ESP_LOGI(MCP_TAG, "Index: %d, Address: 0x%02X, Value: 0x%02X", i,
//              reg->getAddress(), reg->getValue());
//   }
// }

// std::array<std::unique_ptr<MCP::MCPRegister>, MCP::MAX_REG_PER_PORT>
//     registersPortA;
// std::array<std::unique_ptr<MCP::MCPRegister>, MCP::MAX_REG_PER_PORT>
//     registersPortB;

// static void updateRegisters(MCPDevice *device) {
//   if (device) {
//     for (size_t i = 0; i < MCP::MAX_REG_PER_PORT; ++i) {
//       device->registersPortA[i]->updateRegisterAddress();
//       device->registersPortB[i]->updateRegisterAddress();
//     }
//   }
// }

// void initializeRegisters(MCP::PORT port, //
//                          std::array<std::unique_ptr<MCP::MCPRegister>,
//                                     MCP::MAX_REG_PER_PORT> &registers,
//                          bool bankMode) {

//   for (size_t i = 0; i < MCP::MAX_REG_PER_PORT; ++i) {
//     auto regEnum = static_cast<MCP::REG>(i);
//     registers[i] =
//         std::make_unique<MCP::MCPRegister>(model_, regEnum, port, bankMode);
//   }
// }

// static MCP::REG enumTypeFromValue(uint8_t value) {
//   return static_cast<MCP::REG>(value);
// }

// MCP::MCPRegister *getRegister(MCP::REG reg, MCP::PORT port) {
//   auto &registers =
//       (port == MCP::PORT::GPIOA) ? registersPortA : registersPortB;

//   size_t index = static_cast<size_t>(reg);
//   if (index >= MCP::MAX_REG_PER_PORT) {
//     ESP_LOGE(MCP_TAG, "Invalid register index: %d", index);
//     return nullptr;
//   }

//   return registers[index].get();
// }

// MCP::MCPRegister *getRegister(uint8_t reg, MCP::PORT port) {
//   MCP::REG enumReg = enumTypeFromValue(reg);

//   return getRegister(enumReg, port);
// }