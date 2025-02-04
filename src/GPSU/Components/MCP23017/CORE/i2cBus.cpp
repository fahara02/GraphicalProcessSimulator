#include "i2cBus.hpp"
#define I2C_BUS "I2C_BUS"
namespace MCP {
SemaphoreHandle_t I2CBus::i2cMutex = xSemaphoreCreateMutex();

I2CBus::I2CBus(uint8_t addr, int sda, int scl)
    : address_(addr), sda_(sda), scl_(scl),
      wire_(std::make_unique<TwoWire>(0)) {}

void I2CBus::init() {
  if (!wire_->begin(sda_, scl_, 100000)) {
    ESP_LOGE(I2C_BUS, "I2C initialization failed!");
  }
}
void I2CBus::setPin(int sda, int scl) {
  sda_ = sda;
  scl_ = scl;
}

int I2CBus::read_mcp_register(const uint8_t reg, bool bankMode) {

  if (xSemaphoreTake(i2cMutex, I2C_MUTEX_TIMEOUT) != pdTRUE) {
    xSemaphoreGive(i2cMutex);
    return -1;
  }
  uint8_t bytesToRead = 1;
  uint8_t regAddress = reg;

  if (!bankMode) {
    // Default 16 bit mapping
    bytesToRead = 2;
    if ((reg % 2) != 0) {
      regAddress = reg - 1; // Align with Port A
    }
  }

  wire_->beginTransmission(address_);
  wire_->write(regAddress);
  wire_->endTransmission(false);
  wire_->requestFrom((uint8_t)address_, bytesToRead, (uint8_t) true);

  while (wire_->available() < bytesToRead)
    ;

  uint8_t low = wire_->read();
  uint8_t high = (bytesToRead == 2) ? wire_->read() : 0;
  xSemaphoreGive(i2cMutex);

  return (bytesToRead == 2) ? ((high << 8) | low) : low;
}

int I2CBus::write_mcp_register(const uint8_t reg, uint16_t value,
                               bool bankMode) {
  if (xSemaphoreTake(i2cMutex, I2C_MUTEX_TIMEOUT) != pdTRUE) {
    xSemaphoreGive(i2cMutex);
    return -1;
  }
  uint8_t result = 0;
  uint8_t regAddress = reg; // Default: Single register
  uint8_t bytesToWrite = 1; //  8-bit mode

  if (!bankMode) {
    // Default 16 Bit mode
    bytesToWrite = 2;

    // Only adjust for Port B if in 16-bit mode
    if ((reg % 2) != 0) {
      regAddress = reg - 1; // Align to Port A
    }
  }

  wire_->beginTransmission(address_);
  wire_->write(regAddress);
  wire_->write(value & 0xFF); // Write low byte (Port A)
  if (bytesToWrite == 2) {
    wire_->write((value >> 8) & 0xFF); // Write high byte (Port B)
  }
  result = wire_->endTransmission(true);
  xSemaphoreGive(i2cMutex);
  return result;
}

} // namespace MCP