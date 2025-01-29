#ifndef MCP_I2C_BUS_HPP
#define MCP_I2C_BUS_HPP
#include "MCP_Constants.hpp"
#include "cstring"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp32-hal-i2c.h"
#include "esp_log.h"
#include "functional"

#define I2C_BUS "I2C_BUS"
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
namespace MCP {
static constexpr uint16_t MAX_CHUNK_SIZE = 22;
static constexpr uint16_t MIN_CHUNK_SIZE = 2;

class I2Cbus {
private:
  MCP_MODEL model_;
  int sda_;
  int scl_;

  int reset_;
  uint32_t i2c_clk_;
  TickType_t i2c_timeout_;
  i2c_port_t i2c_bus_no;
  bool sequentialMode_;
  uint8_t deviceAddress_;
  uint8_t readBuffer_[MAX_CHUNK_SIZE] = {0};
  size_t bufferSize_ = 0;

  // Read callback function
  std::function<void(uint8_t *, size_t)> readCallback_;

public:
  I2Cbus(MCP_MODEL model);
  void init_gpio() {}
  void init();
  void setDeviceAddress(uint8_t address);
  void setPin(int sda, int scl);
  void setResetPin(int reset);
  void setup_i2c_config(uint32_t frq, uint16_t timeout, int busno);
  void setSequentialMode(bool seqMode);

  esp_err_t i2c_write_byte_mode(const uint8_t reg, uint8_t value);
  esp_err_t i2c_write_sequential_mode(const uint8_t startReg, uint8_t *data,
                                      size_t length);
  esp_err_t i2c_read_byte_mode(const uint8_t reg, uint8_t *buffer);
  esp_err_t i2c_read_sequential_mode(const uint8_t startReg, uint8_t *buffer,
                                     size_t length);

  void setReadCallback(std::function<void(uint8_t *, size_t)> callback);
};

} // namespace MCP
#endif