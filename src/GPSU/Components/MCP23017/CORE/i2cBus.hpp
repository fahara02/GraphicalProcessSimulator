#ifndef I2C_BUS_HPP
#define I2C_BUS_HPP
#include "Arduino.h"
#include "MCP_Constants.hpp"

#include "Wire.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <memory>

namespace MCP {

class I2CBus {

public:
  static I2CBus &getInstance(uint8_t addr, int sda = 25, int scl = 33) {
    static I2CBus instance{addr, sda, scl};

    return instance;
  }
  void setPin(int sda, int scl);
  void init();
  I2CBus(const I2CBus &) = delete;
  I2CBus &operator=(const I2CBus &) = delete;
  I2CBus(I2CBus &&) = delete;
  I2CBus &operator=(I2CBus &&) = delete;

  int read_mcp_register(const uint8_t reg, bool bankMode);
  int write_mcp_register(const uint8_t reg, uint16_t value, bool bankMode);
  void read_mcp_registers_batch(uint8_t startReg, uint8_t *data, size_t length,
                                bool bankMode);
  void write_mcp_registers_batch(uint8_t startReg, const uint8_t *data,
                                 size_t length, bool bankMode);
  static SemaphoreHandle_t i2cMutex;

private:
  I2CBus(uint8_t addr, int sda, int scl);
  uint8_t address_;
  int sda_;
  int scl_;
  std::unique_ptr<TwoWire> wire_;
};

} // namespace MCP
#endif