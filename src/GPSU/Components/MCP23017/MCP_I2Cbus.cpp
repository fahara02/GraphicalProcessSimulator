#include "MCP_I2Cbus.hpp"
namespace MCP {

I2Cbus::I2Cbus(MCP_MODEL model)
    : model_(model), sda_(21), //
      scl_(22),                //
      reset_(33), i2c_clk_(DEFAULT_I2C_CLK_FRQ),
      i2c_timeout_(DEFAULT_I2C_TIMEOUT), i2c_bus_no(I2C_NUM_0),
      sequentialMode_(false) {

  if (model_ == MCP_MODEL::MCP23017) {
    i2c_clk_ = static_cast<uint16_t>(MCP_23X17::MCP_23017::I2C_CLK::CLK_STD);
  } else if (model_ == MCP_MODEL::MCP23S17) {
  }
}
void I2Cbus::init() {
  i2c_config_t config = {};
  config.mode = I2C_MODE_MASTER;
  config.sda_io_num = sda_;
  config.scl_io_num = scl_;
  config.sda_pullup_en = GPIO_PULLUP_ENABLE;
  config.scl_pullup_en = GPIO_PULLUP_ENABLE;
  config.master.clk_speed = i2c_clk_;

  i2c_param_config(i2c_bus_no, &config);
  i2c_driver_install(i2c_bus_no, config.mode, I2C_MASTER_RX_BUF_DISABLE,
                     I2C_MASTER_TX_BUF_DISABLE, 0);
}
void I2Cbus::setReadCallback(std::function<void(uint8_t *, size_t)> callback) {
  readCallback_ = callback;
}

void I2Cbus::setDeviceAddress(uint8_t address) { deviceAddress_ = address; }
void I2Cbus::setPin(int sda, int scl) {
  sda_ = sda;
  scl_ = scl;
}
void I2Cbus::setResetPin(int reset) { reset_ = reset; }
void I2Cbus::setup_i2c_config(uint32_t frq, uint16_t timeout, int busno) {
  if (frq < static_cast<uint32_t>(MCP_23X17::MCP_23017::I2C_CLK::CLK_STD)) {
    ESP_LOGE(I2C_BUS, "Invalid clk frequency given  ");
    i2c_clk_ = static_cast<uint32_t>(MCP_23X17::MCP_23017::I2C_CLK::CLK_STD);
  } else if (frq >
             static_cast<uint32_t>(MCP_23X17::MCP_23017::I2C_CLK::CLK_MAX)) {
    ESP_LOGE(I2C_BUS, "Invalid clk frequency given  ");
    i2c_clk_ = static_cast<uint32_t>(MCP_23X17::MCP_23017::I2C_CLK::CLK_MAX);
  } else {
    i2c_clk_ = frq;
  }

  i2c_timeout_ = static_cast<TickType_t>(timeout);
  if (busno == 0) {
    i2c_bus_no = I2C_NUM_0;
  } else if (busno == 1) {
    i2c_bus_no = I2C_NUM_1;
  } else {
    ESP_LOGE(I2C_BUS, "Invalid I2C Bus number given resetting to 0 ");
    i2c_bus_no = I2C_NUM_0;
  }
};
void I2Cbus::setSequentialMode(bool seqMode) { sequentialMode_ = seqMode; };

esp_err_t I2Cbus::i2c_write_sequential_mode(const uint8_t startReg,
                                            uint8_t *data, size_t length) {
  if (length == 0)
    return ESP_OK;
  esp_err_t ret = ESP_OK;
  size_t remaining = length;
  size_t offset = 0;
  // initiate I2C
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  // Send control byte with write command
  i2c_master_write_byte(cmd, (deviceAddress_ << 1) | I2C_MASTER_WRITE, true);
  // Send starting register address(only for first chunk)
  if (offset == 0) {
    i2c_master_write_byte(cmd, startReg, true);
  }
  while (remaining > 0) {
    size_t chunk_size =
        (remaining > MAX_CHUNK_SIZE) ? MAX_CHUNK_SIZE : remaining;
    // Send data chunk ( MCP auto-increments register address with  each ACK)
    for (size_t i = 0; i < chunk_size; i++) {
      i2c_master_write_byte(cmd, data[offset + i], true);
    }
    // MAX CHUNK SIZE DONE IF MORE REMIANING THEN STOP
    i2c_master_stop(cmd);
    // Execute
    ret = i2c_master_cmd_begin(i2c_bus_no, cmd, i2c_timeout_);
    // DELETE  CMD
    i2c_cmd_link_delete(cmd);

    offset += chunk_size;
    remaining -= chunk_size;
  }
  return ret;
}
esp_err_t I2Cbus::i2c_write_byte_mode(const uint8_t reg, uint8_t value) {
  esp_err_t ret;
  // initiate I2C
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  // Send control byte with write command
  i2c_master_write_byte(cmd, (deviceAddress_ << 1) | I2C_MASTER_WRITE, true);
  // Send starting register address
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_write_byte(cmd, value, true);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_bus_no, cmd, i2c_timeout_);
  i2c_cmd_link_delete(cmd);

  return ret;
}
esp_err_t I2Cbus::i2c_read_byte_mode(const uint8_t reg, uint8_t *buffer) {
  if (buffer == nullptr)
    return ESP_ERR_INVALID_ARG;

  esp_err_t ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (deviceAddress_ << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);

  i2c_master_start(cmd); // Repeated start
  i2c_master_write_byte(cmd, (deviceAddress_ << 1) | I2C_MASTER_READ, true);
  i2c_master_read_byte(cmd, buffer, I2C_MASTER_NACK);

  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_bus_no, cmd, i2c_timeout_);
  i2c_cmd_link_delete(cmd);

  return ret;
}
esp_err_t I2Cbus::i2c_read_sequential_mode(const uint8_t startReg,
                                           uint8_t *buffer, size_t length) {
  if (buffer == nullptr || length == 0)
    return ESP_ERR_INVALID_ARG;

  esp_err_t ret;
  size_t remaining = length;
  size_t offset = 0;

  while (remaining > 0) {
    size_t chunk_size =
        (remaining > MAX_CHUNK_SIZE) ? MAX_CHUNK_SIZE : remaining;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceAddress_ << 1) | I2C_MASTER_WRITE, true);
    if (offset == 0) {
      i2c_master_write_byte(cmd, startReg, true);
    }

    i2c_master_start(cmd); // Repeated start
    i2c_master_write_byte(cmd, (deviceAddress_ << 1) | I2C_MASTER_READ, true);

    for (size_t i = 0; i < chunk_size - 1; i++) {
      i2c_master_read_byte(cmd, &buffer[offset + i], I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, &buffer[offset + chunk_size - 1],
                         I2C_MASTER_NACK);

    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(i2c_bus_no, cmd, i2c_timeout_);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
      return ret;
    }

    offset += chunk_size;
    remaining -= chunk_size;
  }

  // Store in internal buffer
  memcpy(readBuffer_, buffer, length);
  bufferSize_ = length;

  // Trigger callback if set
  if (readCallback_) {
    readCallback_(readBuffer_, bufferSize_);
  }

  return ESP_OK;
}

} // namespace MCP