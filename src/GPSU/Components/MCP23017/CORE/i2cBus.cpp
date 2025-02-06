#include "i2cBus.hpp"
#define I2C_BUS "I2C_BUS"
namespace MCP {
// SemaphoreHandle_t I2CBus::i2cMutex = xSemaphoreCreateMutex();
SemaphoreHandle_t MCP::I2CBus::i2cMutex = nullptr;
I2CBus::I2CBus(uint8_t addr, int sda, int scl)
    : address_(addr), sda_(sda), scl_(scl),
      wire_(std::make_unique<TwoWire>(0)) {
  initMutex();
}

void I2CBus::init() {

  if (!wire_->begin(sda_, scl_, 100000)) {
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
      initialized = true; // ✅ Prevent multiple initializations
    }
  }
}

void I2CBus::setPin(int sda, int scl) {
  sda_ = sda;
  scl_ = scl;
}

int I2CBus::read_mcp_register(const uint8_t reg, bool map8Bit) {

  // if (xSemaphoreTake(i2cMutex, I2C_MUTEX_TIMEOUT) != pdTRUE) {

  //   ESP_LOGE("I2CBUS", "Failed to take mutex");
  //   return -1;
  // }
  I2CLock lock(i2cMutex, I2C_MUTEX_TIMEOUT);
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
  // xSemaphoreGive(i2cMutex);

  return (bytesToRead == 2) ? ((highByte << 8) | (lowByte)) : lowByte;
}
int I2CBus::write_mcp_register(const uint8_t reg, uint16_t value,
                               bool map8Bit) {
  // if (xSemaphoreTake(i2cMutex, I2C_MUTEX_TIMEOUT) != pdTRUE) {
  //   // Don’t call xSemaphoreGive here since we never took it.
  //   ESP_LOGE("I2CBUS", "Failed to take mutex");
  //   return -1;
  // }
  I2CLock lock(i2cMutex, I2C_MUTEX_TIMEOUT);
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
  // xSemaphoreGive(i2cMutex);
  if (result != 0) {
    ESP_LOGE("I2CBUS", "I2C transmission failed with error code: %d", result);
  }
  return result;
}

// int I2CBus::write_mcp_register(const uint8_t reg, uint16_t value,
//                                bool map8Bit) {
//   if (xSemaphoreTake(i2cMutex, I2C_MUTEX_TIMEOUT) != pdTRUE) {

//     ESP_LOGE("I2CBUS", "Failed to take mutex");
//     return -1;
//   }
//   int result = 0;
//   uint8_t regAddress = reg; // Default: Single register
//   uint8_t bytesToWrite = 1; //  8-bit mode

//   if (!map8Bit) {
//     // Default 16 Bit mode
//     bytesToWrite = 2;

//     // Only adjust for Port B if in 16-bit mode
//     if ((reg % 2) != 0) {
//       regAddress = reg - 1; // Align to Port A
//     }
//   }

//   wire_->beginTransmission(address_);
//   wire_->write(regAddress);
//   wire_->write(value & 0xFF); // Write low byte (Port A)
//   if (bytesToWrite == 2) {
//     wire_->write((value >> 8) & 0xFF); // Write high byte (Port B)
//   }
//   result = wire_->endTransmission(true);
//   xSemaphoreGive(i2cMutex);
//   return result;
// }
// void I2CBus::write_mcp_registers_batch(uint8_t startReg, const uint8_t *data,
//                                        size_t length, bool bankMode) {
//   if (!data || length == 0) {
//     Serial.println("Invalid data or length for batch write.");
//     return;
//   }

//   if (xSemaphoreTake(i2cMutex, I2C_MUTEX_TIMEOUT)) {
//     wire_->beginTransmission(address_);
//     wire_->write(startReg); // Start at the first register address

//     for (size_t i = 0; i < length; ++i) {
//       wire_->write(data[i]); // Write each byte
//     }

//     esp_err_t result = wire_->endTransmission(true);
//     if (result == ESP_OK) {
//       Serial.printf("Batch write successful: %d bytes starting at 0x%X.\n",
//                     length, startReg);
//     } else {
//       Serial.printf("Batch write failed with error: %d.\n", result);
//     }

//     xSemaphoreGive(i2cMutex);
//   } else {
//     Serial.println("Failed to acquire mutex for batch write.");
//   }
// }

// void I2CBus::read_mcp_registers_batch(uint8_t startReg, uint8_t *buffer,
//                                       size_t length, bool bankMode) {
//   if (!buffer || length == 0) {
//     Serial.println("Invalid buffer or length for batch read.");
//     return;
//   }

//   if (xSemaphoreTake(i2cMutex, I2C_MUTEX_TIMEOUT)) {
//     wire_->beginTransmission(address_);
//     wire_->write(startReg);        // Start at the first register address
//     wire_->endTransmission(false); // Restart condition to read

//     size_t bytesRead =
//         wire_->requestFrom((uint8_t)address_, (uint8_t)length, (uint8_t)
//         true);
//     if (bytesRead == length) {
//       for (size_t i = 0; i < length; ++i) {
//         buffer[i] = wire_->read(); // Read each byte into buffer
//       }
//       Serial.printf("Batch read successful: %d bytes starting at 0x%X.\n",
//                     length, startReg);
//     } else {
//       Serial.printf("Batch read failed. Expected %d bytes, got %d.\n",
//       length,
//                     bytesRead);
//     }

//     xSemaphoreGive(i2cMutex);
//   } else {
//     Serial.println("Failed to acquire mutex for batch read.");
//   }
// }

} // namespace MCP

// std::unordered_map<uint8_t, uint8_t> MCPDevice::batchReadRegisters() {
//   std::unordered_map<uint8_t, uint8_t> registerValues;
//   constexpr size_t totalRegisters = 2 * MCP::MAX_REG_PER_PORT;

//   uint8_t buffer[totalRegisters] = {0};

//   uint8_t startRegAddress =
//       Util::calculateAddress(MCP::REG::IODIR, MCP::PORT::GPIOA, bankMode_);
//   read_mcp_registers_batch(startRegAddress, buffer, totalRegisters);

//   // Map the buffer data to register addresses
//   for (uint8_t i = 0; i < totalRegisters; ++i) {
//     // Calculate the corresponding register address
//     MCP::PORT port =
//         (i < MCP::MAX_REG_PER_PORT) ? MCP::PORT::GPIOA : MCP::PORT::GPIOB;
//     MCP::REG reg = static_cast<MCP::REG>(i % MCP::MAX_REG_PER_PORT);

//     uint8_t address = Util::calculateAddress(reg, port, bankMode_);
//     registerValues[address] = buffer[i];
//   }

//   return registerValues;
// }