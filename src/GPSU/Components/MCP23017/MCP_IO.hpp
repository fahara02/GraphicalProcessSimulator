#ifndef MCP_IO_HPP
#define MCP_IO_HPP

#include "MCP_Defines.hpp"
#include "Wire.h"
#include "driver/gpio.h"
#include <memory>
#include <variant>

namespace COMPONENT {
using MCP23017Device =
    MCP::MCPDevice<MCP::MCP_23X17::REG, MCP::MCP_MODEL::MCP23017>;
using MCP23S17Device =
    MCP::MCPDevice<MCP::MCP_23X17::REG, MCP::MCP_MODEL::MCP23S17>;

class MCP_IO_EXPANDER {

public:
  MCP_IO_EXPANDER(uint8_t address,
                  MCP::MCP_MODEL model = MCP::MCP_MODEL::MCP23017)
      : model_(model), address_(address), //
        sda_(GPIO_NUM_21),                //
        scl_(GPIO_NUM_22),                //
        cs_(GPIO_NUM_NC),                 //
        reset_(GPIO_NUM_33),              //
        wire_(std::make_unique<TwoWire>(0)), mcp_device(initDevice(model)) {}

private:
  MCP::MCP_MODEL model_; // MCP model (e.g., MCP23017)
  uint8_t address_;
  gpio_num_t sda_;                // SDA GPIO pin
  gpio_num_t scl_;                // SCL GPIO pin
  gpio_num_t cs_;                 // Chip Select GPIO pin
  gpio_num_t reset_;              // Reset GPIO pin
  std::unique_ptr<TwoWire> wire_; // Unique pointer for TwoWire instance

  using DeviceType = std::variant<MCP23017Device, MCP23S17Device>;
  DeviceType mcp_device;

  static DeviceType initDevice(MCP::MCP_MODEL model) {
    if (model == MCP::MCP_MODEL::MCP23017) {
      return MCP23017Device();
    } else {
      return MCP23S17Device();
    }
  }
  void init();
  void setup(MCP::register_icon_t &config);
};

} // namespace COMPONENT

#endif // MCP23017_HPP
