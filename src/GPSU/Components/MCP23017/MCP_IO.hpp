#ifndef MCP_IO_HPP
#define MCP_IO_HPP

#include "MCP_Defines.hpp"
#include "Wire.h"
#include "driver/gpio.h"
#include <memory>

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
        wire_(std::make_unique<TwoWire>(0)) {}

private:
  MCP::MCP_MODEL model_; // MCP model (e.g., MCP23017)
  uint8_t address_;
  gpio_num_t sda_;                // SDA GPIO pin
  gpio_num_t scl_;                // SCL GPIO pin
  gpio_num_t cs_;                 // Chip Select GPIO pin
  gpio_num_t reset_;              // Reset GPIO pin
  std::unique_ptr<TwoWire> wire_; // Unique pointer for TwoWire instance
  MCP::register_icon_t config_reg_icon_;
  void init();
  void setup(MCP::register_icon_t &config);
};

} // namespace COMPONENT

#endif // MCP23017_HPP
