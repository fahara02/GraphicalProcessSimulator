#include "MCPDevice.hpp"
#include "climits"
using namespace MCP;
namespace COMPONENT {
SemaphoreHandle_t MCPDevice::regRWmutex = xSemaphoreCreateMutex();
MCPDevice::MCPDevice(MCP::MCP_MODEL model, bool pinA2, bool pinA1, bool pinA0)
    : model_(model), configuration_(model),
      settings_(configuration_.getSettings()),
      defaultSettings_(Settings(model)), decoder_(model, pinA2, pinA1, pinA0),
      address_(decoder_.getDeviceAddress()), //
      sda_(GPIO_NUM_25),                     //
      scl_(GPIO_NUM_33),                     //
      cs_(GPIO_NUM_NC),                      //
      reset_(GPIO_NUM_33),                   //
      wire_(std::make_unique<TwoWire>(1)),   //
      gpioBankA(std::make_unique<MCP::GPIO_BANK>(MCP::PORT::GPIOA, model)),
      gpioBankB(std::make_unique<MCP::GPIO_BANK>(MCP::PORT::GPIOB, model)),
      cntrlRegA(gpioBankA->getControlRegister()),
      cntrlRegB(gpioBankB->getControlRegister())

{
  init();
  loadSettings();
}
void MCPDevice::configure(const MCP::Settings &setting) {
  if (cntrlRegA && cntrlRegB) {
    if (configuration_.configure(setting)) {
      settings_ = configuration_.getSettings();
      loadSettings();
    }
  }
}
void MCPDevice::loadSettings() {

  if (settings_ != defaultSettings_) {

    if (settings_.opMode == MCP::OperationMode::SequentialMode16 ||
        settings_.opMode == MCP::OperationMode::ByteMode16) {
      bankMode_ = true; // 8bitMapping
    }

    if (settings_.opMode == MCP::OperationMode::ByteMode16 ||
        settings_.opMode == MCP::OperationMode::ByteMode8) {
      byteMode_ = true; // Adress pointer dont increment ,continous poll
    }
    mirrorMode_ =
        settings_.mirror == MCP::PairedInterrupt::Enabled ? true : false;
    slewrateDisabled_ = settings_.slew == MCP::Slew::Disabled ? true : false;

    hardwareAddressing_ =
        settings_.haen == MCP::HardwareAddr::Enabled ? true : false;

    opendrainEnabled_ = settings_.odr == MCP::OpenDrain::Enabled ? true : false;
    interruptPolarityHigh_ =
        settings_.intpol == MCP::InterruptPolarity::ActiveHigh ? true : false;

    // Send the Settings To MCP Register
    uint8_t updatedSetting = configuration_.getSettingValue();

    uint8_t result = 0;
    if (bankMode_) {
      // Delay the Register address update before sending as
      // default is bankMode= false i.e 16 bit mapping address
      // write 8 bit value to both port
      result |= write_mcp_register(cntrlRegA->getAddress(), updatedSetting);
      result |= write_mcp_register(cntrlRegB->getAddress(), updatedSetting);

      gpioBankA->updateBankMode(bankMode_);
      gpioBankB->updateBankMode(bankMode_);
    } else {
      // if not writing 16 bit to A port is enough and no address change

      result |= write_mcp_register(cntrlRegA->getAddress(), updatedSetting);
      result |= write_mcp_register(cntrlRegA->getAddress() + 1, updatedSetting);
    }
    cntrlRegA->configure<MCP::REG::IOCON>(updatedSetting);
    cntrlRegB->configure<MCP::REG::IOCON>(updatedSetting);
    if (result != 0) {
      ESP_LOGE(MCP_TAG, "new_Setting changed failed , going back to defaults");
      configuration_.configureDefault();
    } else {
      ESP_LOGI(MCP_TAG, "succefully changed the settings");
    }
  }
}
void MCPDevice::resetDevice() { ESP_LOGI(MCP_TAG, "resetting the device"); }
void MCPDevice::init() {
  if (!wire_->begin(sda_, scl_, 100000)) {
    ESP_LOGE(MCP_TAG, "i2c init failed");
  }

  EventManager::initializeEventGroups();

  startEventMonitorTask(this);
}
void MCPDevice::startEventMonitorTask(MCPDevice *device) {
  if (!device) {
    ESP_LOGE(MCP_TAG, "no_device");
  } else {
    xTaskCreatePinnedToCore(EventMonitorTask, "EventMonitorTask", 8192, device,
                            5, &eventTaskHandle, 0);
  }
}

void MCPDevice::pinMode(MCP::Pin pin, const uint8_t mode) {

  PIN pinEnum = pin.getEnum();
  uint8_t pinIndex = Util::getPinIndex(pinEnum);
  return pinMode(pinIndex, mode);
}



void MCPDevice::pinMode(const int pin, const uint8_t mode) {
  MCP::PIN pinEnum;
  if (0 <= pin && pin <= 15) {
    pinEnum = static_cast<MCP::PIN>(pin);
  } else {
    assert(false && "Invalid pin");
    return;
  }
  MCP::PORT port = Util::getPortFromPin(pinEnum);

  GPIO_BANK *gpioBank =
      (port == MCP::PORT::GPIOA) ? gpioBankA.get() : gpioBankB.get();
  auto *cntrlReg =
      (port == MCP::PORT::GPIOA) ? cntrlRegA.get() : cntrlRegB.get();

  if (!gpioBank) {
    assert(false && "Invalid port");
    return;
  }

  switch (mode) {
  case INPUT:
    gpioBank->setPinDirection(pinEnum, MCP::GPIO_MODE::GPIO_INPUT);
    break;
  case INPUT_PULLUP:
    gpioBank->setPinDirection(pinEnum, MCP::GPIO_MODE::GPIO_INPUT);
    gpioBank->setPullup(pinEnum, MCP::PULL_MODE::ENABLE_PULLUP);
    break;
  case INPUT_PULLDOWN:

    ESP_LOGE(MCP_TAG,
             "PullDown not available in MCP devices, defaulting to INPUT.");
    break;
  case OUTPUT:
    gpioBank->setPinDirection(pinEnum, MCP::GPIO_MODE::GPIO_OUTPUT);
    break;
  case OUTPUT_OPEN_DRAIN:
    gpioBank->setPinDirection(pinEnum, MCP::GPIO_MODE::GPIO_OUTPUT);
    cntrlReg->setOpenDrain<MCP::REG::IOCON>(true);
    break;
  default:
    assert(false && "Invalid mode");
    break;
  }
}
void MCPDevice::pinMode(MCP::PORT port, uint8_t pinmask, const uint8_t mode) {

  GPIO_BANK *gpioBank =
      (port == MCP::PORT::GPIOA) ? gpioBankA.get() : gpioBankB.get();
  auto *cntrlReg =
      (port == MCP::PORT::GPIOA) ? cntrlRegA.get() : cntrlRegB.get();
  if (!gpioBank) {
    assert(false && "Invalid port");
    return;
  }

  switch (mode) {
  case INPUT:
    gpioBank->setPinDirection(pinmask, MCP::GPIO_MODE::GPIO_INPUT);
    break;
  case INPUT_PULLUP:
    gpioBank->setPinDirection(pinmask, MCP::GPIO_MODE::GPIO_INPUT);
    gpioBank->setPullup(pinmask, MCP::PULL_MODE::ENABLE_PULLUP);
    break;
  case INPUT_PULLDOWN:

    ESP_LOGE(MCP_TAG,
             "PullDown not available in MCP devices, defaulting to INPUT.");
    break;
  case OUTPUT:
    gpioBank->setPinDirection(pinmask, MCP::GPIO_MODE::GPIO_OUTPUT);
    break;
  case OUTPUT_OPEN_DRAIN:
    gpioBank->setPinDirection(pinmask, MCP::GPIO_MODE::GPIO_OUTPUT);
    cntrlReg->setOpenDrain<MCP::REG::IOCON>(true);
    break;
  default:
    assert(false && "Invalid mode");
    break;
  }
}
void MCPDevice::pinMode(MCP::PORT port, const uint8_t mode) {

  GPIO_BANK *gpioBank =
      (port == MCP::PORT::GPIOA) ? gpioBankA.get() : gpioBankB.get();
  auto *cntrlReg =
      (port == MCP::PORT::GPIOA) ? cntrlRegA.get() : cntrlRegB.get();
  if (!gpioBank) {
    assert(false && "Invalid port");
    return;
  }

  switch (mode) {
  case INPUT:
    gpioBank->setPinDirection(MCP::GPIO_MODE::GPIO_INPUT);
    break;
  case INPUT_PULLUP:
    gpioBank->setPinDirection(MCP::GPIO_MODE::GPIO_INPUT);
    gpioBank->setPullup(MCP::PULL_MODE::ENABLE_PULLUP);
    break;
  case INPUT_PULLDOWN:

    ESP_LOGE(MCP_TAG,
             "PullDown not available in MCP devices, defaulting to INPUT.");
    break;
  case OUTPUT:
    gpioBank->setPinDirection(MCP::GPIO_MODE::GPIO_OUTPUT);
    break;
  case OUTPUT_OPEN_DRAIN:
    gpioBank->setPinDirection(MCP::GPIO_MODE::GPIO_OUTPUT);
    cntrlReg->setOpenDrain<MCP::REG::IOCON>(true);
    break;
  default:
    assert(false && "Invalid mode");
    break;
  }
}

int MCPDevice::read_mcp_register(const uint8_t reg) {
  uint8_t bytesToRead = 1;
  uint8_t regAddress = reg;

  if (!bankMode_) {
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

uint8_t MCPDevice::write_mcp_register(const uint8_t reg, uint16_t value) {
  uint8_t result = 0;
  uint8_t regAddress = reg; // Default: Single register
  uint8_t bytesToWrite = 1; //  8-bit mode

  if (!bankMode_) {
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

void MCPDevice::EventMonitorTask(void *param) {
  MCPDevice *device = static_cast<MCPDevice *>(param);

  while (true) {
    // Wait for events
    size_t queueSize = EventManager::getQueueSize();
    if (MAX_EVENT - queueSize < 2) {

      if (xSemaphoreTake(regRWmutex, MUTEX_TIMEOUT)) {
        // EventManager::clearOldestEvent();
        Serial.printf("QUE Cleared");
      } else {
        Serial.printf(" Monitor task: Failed to get mutex!");
      }

      xSemaphoreGive(regRWmutex);
    }

    const EventBits_t CHECK_BITS_MASK =
        static_cast<EventBits_t>(RegisterEvent::READ_REQUEST) |
        static_cast<EventBits_t>(RegisterEvent::WRITE_REQUEST) |
        static_cast<EventBits_t>(RegisterEvent::BANK_MODE_CHANGED) |
        static_cast<EventBits_t>(RegisterEvent::SETTINGS_CHANGED);
    EventBits_t eventBits =
        xEventGroupWaitBits(EventManager::registerEventGroup, CHECK_BITS_MASK,

                            pdFALSE, // Clear the bits after processing
                            pdFALSE, // Wait for any event
                            MUTEX_TIMEOUT);

    // Handle READ_REQUEST

    if (eventBits & static_cast<EventBits_t>(RegisterEvent::READ_REQUEST)) {

      if (xSemaphoreTake(regRWmutex, MUTEX_TIMEOUT)) {
        currentEvent *event =
            EventManager::getEvent(RegisterEvent::READ_REQUEST);
        if (event && event->event != RegisterEvent::MAX) {

          device->handleReadEvent(event);
        }
      } else {
        Serial.printf(" Monitor task(read): Failed to get mutex!");
      }
    }

    // Handle WRITE_REQUEST
    if (eventBits & static_cast<EventBits_t>(RegisterEvent::WRITE_REQUEST)) {
      if (xSemaphoreTake(regRWmutex, MUTEX_TIMEOUT)) {
        currentEvent *event =
            EventManager::getEvent(RegisterEvent::WRITE_REQUEST);
        if (event && event->event != RegisterEvent::MAX) {

          device->handleWriteEvent(event);
        }
      } else {
        Serial.printf(" Monitor task(write): Failed to get mutex!");
      }
    }

    // Handle BANK_MODE_CHANGED
    if (eventBits &
        static_cast<EventBits_t>(RegisterEvent::BANK_MODE_CHANGED)) {
      if (xSemaphoreTake(regRWmutex, MUTEX_TIMEOUT)) {
        currentEvent *event =
            EventManager::getEvent(RegisterEvent::BANK_MODE_CHANGED);
        if (event && event->event != RegisterEvent::MAX) {

          device->handleBankModeEvent(event);
        }
      } else {
        Serial.printf(" Monitor task: Failed to get mutex!");
      }
    }

    // Handle SETTINGS_CHANGED
    if (eventBits & static_cast<EventBits_t>(RegisterEvent::SETTINGS_CHANGED)) {
      if (xSemaphoreTake(regRWmutex, MUTEX_TIMEOUT)) {
        currentEvent *event =
            EventManager::getEvent(RegisterEvent::SETTINGS_CHANGED);
        if (event && event->event != RegisterEvent::MAX) {

          device->handleSettingChangeEvent(event);
        }
      } else {
        Serial.printf(" Monitor task (Settings): Failed to get mutex!");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  vTaskDelete(NULL);
}
void MCPDevice::handleBankModeEvent(currentEvent *ev) {

  MCP::PORT port = ev->regIdentity.port;

  uint8_t regAddress = ev->regIdentity.regAddress;
  uint8_t settings = ev->data;
  write_mcp_register(regAddress, settings);

  Serial.printf("New Event %d BankMode changed \n", ev->id);
  EventManager::acknowledgeEvent(ev);
  EventManager::clearBits(RegisterEvent::BANK_MODE_CHANGED);
  xSemaphoreGive(regRWmutex);
}

void MCPDevice::handleReadEvent(currentEvent *ev) {
  uint8_t reg = ev->regIdentity.regAddress;
  MCP::PORT port = ev->regIdentity.port;

  int value = read_mcp_register(reg);
  if (value == -1) {
    Serial.printf("Read failed for id=%d ; Invalid data received\n", ev->id);
    return;
  }

  if (!bankMode_) {
    uint8_t valueA = value & 0xFF;
    uint8_t valueB = (value >> 8) & 0xFF;
    gpioBankA->updateRegisterValue(reg, valueA);
    gpioBankB->updateRegisterValue(reg + 1, valueB);
  } else {
    if (port == MCP::PORT::GPIOA) {
      gpioBankA->updateRegisterValue(reg, value);
    } else {
      gpioBankB->updateRegisterValue(reg, value);
    }
  }

  EventManager::acknowledgeEvent(ev);
  EventManager::clearBits(RegisterEvent::READ_REQUEST);

  EventManager::createEvent(ev->regIdentity, RegisterEvent::DATA_RECEIVED,
                            value);

  xSemaphoreGive(regRWmutex);
}

void MCPDevice::handleWriteEvent(currentEvent *ev) {
  uint8_t reg = ev->regIdentity.regAddress;
  uint16_t value = ev->data; // Use 16-bit storage

  uint8_t result = write_mcp_register(reg, value); // Handles 8-bit & 16-bit

  if (result == 0) {

    EventManager::acknowledgeEvent(ev);
  } else {
    ESP_LOGE(MCP_TAG, "New Write failed for id=%d ; \n", ev->id);
  }

  EventManager::clearBits(RegisterEvent::WRITE_REQUEST);
  xSemaphoreGive(regRWmutex);
}

void MCPDevice::handleSettingChangeEvent(currentEvent *ev) {
  MCP::PORT port = ev->regIdentity.port;
  uint8_t reg = ev->regIdentity.regAddress;
  uint16_t settings = ev->data;
  uint8_t result = write_mcp_register(reg, settings);
  if (result == 0) {
    Serial.printf("New Setting Event sucessfull id=%d ; \n", ev->id);
    EventManager::acknowledgeEvent(ev);
  } else {
    Serial.printf("New Setting Event failed for id=%d ; \n", ev->id);
  }

  EventManager::clearBits(RegisterEvent::SETTINGS_CHANGED);
  xSemaphoreGive(regRWmutex);
}

void MCPDevice::read_mcp_registers_batch(uint8_t startReg, uint8_t *buffer,
                                         size_t length) {
  if (!buffer || length == 0) {
    Serial.println("Invalid buffer or length for batch read.");
    return;
  }

  if (xSemaphoreTake(regRWmutex, MUTEX_TIMEOUT)) {
    wire_->beginTransmission(address_);
    wire_->write(startReg);        // Start at the first register address
    wire_->endTransmission(false); // Restart condition to read

    size_t bytesRead =
        wire_->requestFrom((uint8_t)address_, (uint8_t)length, (uint8_t) true);
    if (bytesRead == length) {
      for (size_t i = 0; i < length; ++i) {
        buffer[i] = wire_->read(); // Read each byte into buffer
      }
      Serial.printf("Batch read successful: %d bytes starting at 0x%X.\n",
                    length, startReg);
    } else {
      Serial.printf("Batch read failed. Expected %d bytes, got %d.\n", length,
                    bytesRead);
    }

    xSemaphoreGive(regRWmutex);
  } else {
    Serial.println("Failed to acquire mutex for batch read.");
  }
}
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
void MCPDevice::write_mcp_registers_batch(uint8_t startReg, const uint8_t *data,
                                          size_t length) {
  if (!data || length == 0) {
    Serial.println("Invalid data or length for batch write.");
    return;
  }

  if (xSemaphoreTake(regRWmutex, MUTEX_TIMEOUT)) {
    wire_->beginTransmission(address_);
    wire_->write(startReg); // Start at the first register address

    for (size_t i = 0; i < length; ++i) {
      wire_->write(data[i]); // Write each byte
    }

    esp_err_t result = wire_->endTransmission(true);
    if (result == ESP_OK) {
      Serial.printf("Batch write successful: %d bytes starting at 0x%X.\n",
                    length, startReg);
    } else {
      Serial.printf("Batch write failed with error: %d.\n", result);
    }

    xSemaphoreGive(regRWmutex);
  } else {
    Serial.println("Failed to acquire mutex for batch write.");
  }
}

MCP::MCPRegister *MCPDevice::getRegister(MCP::REG reg, MCP::PORT port) {
  if (port == MCP::PORT::GPIOA) {

    return gpioBankA->getRegisterForUpdate(reg);

  } else {

    return gpioBankB->getRegisterForUpdate(reg);
  }
}
uint8_t MCPDevice::getsavedSettings(MCP::PORT port) const {
  return port == MCP::PORT::GPIOA ? cntrlRegA->getSavedValue()
                                  : cntrlRegB->getSavedValue();
}

uint8_t MCPDevice::getRegisterAddress(MCP::REG reg, MCP::PORT port) const {
  if (port == MCP::PORT::GPIOA) {
    return gpioBankA->getAddress(reg);
  } else {
    return gpioBankB->getAddress(reg);
  }
}

uint8_t MCPDevice::getRegisterSavedValue(MCP::REG reg, MCP::PORT port) const {
  if (port == MCP::PORT::GPIOA) {
    return gpioBankA->getSavedValue(reg);
  } else {
    return gpioBankB->getSavedValue(reg);
  }
}
void MCPDevice::dumpRegisters() const {

  ESP_LOGI(MCP_TAG, "Dumping Registers for MCP_Device (Address: 0x%02X)",
           address_);

  // Dump PORTA Registers
  ESP_LOGI(MCP_TAG, "PORTA Registers:");
  for (uint8_t i = 0; i < MCP::MAX_REG_PER_PORT; ++i) {
    MCP::REG reg = static_cast<MCP::REG>(i);
    uint8_t address = getRegisterAddress(reg, MCP::PORT::GPIOA);
    uint8_t value = getRegisterSavedValue(reg, MCP::PORT::GPIOA);
    ESP_LOGI(MCP_TAG, "Index: %d, Address: 0x%02X, Value: 0x%02X", i, address,
             value);
  }

  // Dump PORTB Registers
  ESP_LOGI(MCP_TAG, "PORTB Registers:");
  for (uint8_t i = 0; i < MCP::MAX_REG_PER_PORT; ++i) {
    MCP::REG reg = static_cast<MCP::REG>(i);
    uint8_t address = getRegisterAddress(reg, MCP::PORT::GPIOB);
    uint8_t value = getRegisterSavedValue(reg, MCP::PORT::GPIOB);
    ESP_LOGI(MCP_TAG, "Index: %d, Address: 0x%02X, Value: 0x%02X", i, address,
             value);
  }
}
void MCPDevice::updateRegisters(MCPDevice *device) {
  if (device) {
  }
}

} // namespace COMPONENT
