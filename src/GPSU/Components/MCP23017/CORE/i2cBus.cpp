#include "i2cBus.hpp"
#define I2C_BUS "I2C_BUS"
namespace MCP {
SemaphoreHandle_t I2CBus::regRWmutex = xSemaphoreCreateMutex();

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
void I2CBus::startEventMonitorTask(I2CBus *bus) {
  if (!bus) {
    ESP_LOGE(I2C_BUS, "no i2c bus");
  } else {
    xTaskCreatePinnedToCore(EventMonitorTask, "EventMonitorTask", 8192, bus, 5,
                            &eventTaskHandle, 0);
  }
}
int I2CBus::read_mcp_register(const uint8_t reg, bool bankMode) {
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

  return (bytesToRead == 2) ? ((high << 8) | low) : low;
}

uint8_t I2CBus::write_mcp_register(const uint8_t reg, uint16_t value,
                                   bool bankMode) {
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

  return result;
}

} // namespace MCP