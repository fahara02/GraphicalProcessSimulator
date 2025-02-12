#include "i2cBus.hpp"
#define I2C_BUS "I2C_BUS"
namespace MCP {

SemaphoreHandle_t MCP::I2CBus::i2cMutex = nullptr;
I2CBus::I2CBus(uint8_t addr)
    : address_(addr), wire_(std::make_unique<TwoWire>(0)) {
  initMutex();
}

void I2CBus::init() {

  if (!wire_->begin(sda_, scl_, i2cClock_)) {
    ESP_LOGE(I2C_BUS, "I2C initialization failed!");
  }
}
void I2CBus::initMutex() {
  static bool initialized = false;
  if (!initialized) {
    i2cMutex = xSemaphoreCreateMutex();
    if (i2cMutex == nullptr) {
      ESP_LOGE("I2C_BUS", "Failed to create I2C mutex");
    } else {
      initialized = true;
    }
  }
}

void I2CBus::setup(int sda, int scl, uint32_t clock, TickType_t timeout) {

  sda_ = sda;
  scl_ = scl;
  i2cClock_ = clock;
  timeout_ = timeout;
}

int I2CBus::read_mcp_register(const uint8_t reg, bool map8Bit) {

  SemLock lock(i2cMutex, I2C_MUTEX_TIMEOUT);
  if (!lock.acquired()) {
    ESP_LOGE("I2CBUS", "Failed to take mutex");
    return -1;
  }
  uint8_t bytesToRead = 1;
  uint8_t regAddress = reg;

  if (!map8Bit) {
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
  // as address pointer increases first byte is portA
  uint8_t lowByte = wire_->read();                           // PORT A
  uint8_t highByte = (bytesToRead == 2) ? wire_->read() : 0; // PORT B

  return (bytesToRead == 2) ? ((highByte << 8) | (lowByte)) : lowByte;
}
int I2CBus::write_mcp_register(const uint8_t reg, uint16_t value,
                               bool map8Bit) {

  SemLock lock(i2cMutex, I2C_MUTEX_TIMEOUT);
  if (!lock.acquired()) {
    ESP_LOGE("I2CBUS", "Failed to take mutex");
    return -1;
  }
  int result = 0;
  uint8_t regAddress = reg; // Default: single register
  uint8_t bytesToWrite = map8Bit ? 1 : 2;

  wire_->beginTransmission(address_);
  // For 16-bit mode, adjust and determine order based on parity.
  if (bytesToWrite == 2) {
    if ((reg % 2) == 0) {
      // Even: assume reg is Port A; low byte -> A, high byte -> B
      regAddress = reg;
      wire_->write(regAddress);
      wire_->write(value & 0xFF);        // Port A
      wire_->write((value >> 8) & 0xFF); // Port B
    } else {
      // Odd: assume reg is Port B; subtract 1 to get base and swap order.
      regAddress = reg - 1;
      wire_->write(regAddress);
      wire_->write((value >> 8) & 0xFF); // Port B now comes first
      wire_->write(value & 0xFF);        // Port A second
    }
  } else {
    // 8-bit mode: simply write one byte.
    wire_->write(regAddress);
    wire_->write(value & 0xFF);
  }

  result = wire_->endTransmission(true);

  if (result != 0) {
    ESP_LOGE("I2CBUS", "I2C transmission failed with error code: %d", result);
  }
  return result;
}

} // namespace MCP
