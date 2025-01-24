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
  using regEnum = std::variant<regEnumMCP17, regEnumMCP18>;
  void init();
  void setup(const MCP::register_icon_t &config) {
    if (device_) {
      device_->configure(config);
    }
  }
  template <typename RegEnum, MCP::MCP_MODEL Model, typename RetType,
            typename... Args>
  RetType
  invokeMethod(RetType (MCP::MCPDevice<RegEnum, Model>::*method)(Args...),
               Args &&...args) const {
    if (!device_) {
      ESP_LOGE(MCP_TAG, "Device not initialized");
      return RetType();
    }

    return invokeMethodImpl<RegEnum, Model>(method,
                                            std::forward<Args>(args)...);
  }
  template <typename RegEnum, MCP::MCP_MODEL Model, typename RetType,
            typename... Args>
  RetType
  invokeMethodImpl(RetType (MCP::MCPDevice<RegEnum, Model>::*method)(Args...),
                   Args &&...args) const {
    if constexpr (Model == MCP::MCP_MODEL::MCP23017) {
      auto derivedDevice = static_cast<
          const MCP::MCPDevice<RegEnum, MCP::MCP_MODEL::MCP23017> *>(
          device_.get());
      return (derivedDevice->*method)(std::forward<Args>(args)...);
    } else if constexpr (Model == MCP::MCP_MODEL::MCP23S17) {
      auto derivedDevice = static_cast<
          const MCP::MCPDevice<RegEnum, MCP::MCP_MODEL::MCP23S17> *>(
          device_.get());
      return (derivedDevice->*method)(std::forward<Args>(args)...);
    } else if constexpr (Model == MCP::MCP_MODEL::MCP23018) {
      auto derivedDevice = static_cast<
          const MCP::MCPDevice<RegEnum, MCP::MCP_MODEL::MCP23018> *>(
          device_.get());
      return (derivedDevice->*method)(std::forward<Args>(args)...);
    } else if constexpr (Model == MCP::MCP_MODEL::MCP23S18) {
      auto derivedDevice = static_cast<
          const MCP::MCPDevice<RegEnum, MCP::MCP_MODEL::MCP23S18> *>(
          device_.get());
      return (derivedDevice->*method)(std::forward<Args>(args)...);
    } else {
      ESP_LOGE(MCP_TAG, "Unknown device model");
      return RetType(); // Return default-constructed value for unknown model
    }
  }

  template <typename RegEnum, MCP::MCP_MODEL Model>
  uint8_t getRegisterAddress(RegEnum reg, MCP::PORT port) const {
    return invokeMethod<RegEnum, Model, uint8_t>(
        &MCP::MCPDevice<RegEnum, Model>::getAddress, reg, port);
  }

  template <typename RegEnum, MCP::MCP_MODEL Model>
  void configureDevice(const MCP::register_icon_t &config) {
    invokeMethod<RegEnum, Model, void>(
        &MCP::MCPDevice<RegEnum, Model>::configure, config);
  }
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
};

} // namespace COMPONENT

#endif // MCP23017_HPP
