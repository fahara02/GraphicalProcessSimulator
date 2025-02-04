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
      i2cBus_(MCP::I2CBus::getInstance(address_, sda_, scl_)),
      gpioBankA(
          std::make_unique<MCP::GPIO_BANK>(MCP::PORT::GPIOA, model, i2cBus_)),
      gpioBankB(
          std::make_unique<MCP::GPIO_BANK>(MCP::PORT::GPIOB, model, i2cBus_)),
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
  uint8_t result = 0;

  if (settings_ != defaultSettings_) {
    // Determine bankMode_ and byteMode_ at once
    switch (settings_.opMode) {
    case MCP::OperationMode::SequentialMode16: // SEQOP = 0, BANK = 0
      bankMode_ = false;
      byteMode_ = false;
      break;

    case MCP::OperationMode::SequentialMode8: // SEQOP = 0, BANK = 1
      bankMode_ = true;
      byteMode_ = false;
      break;

    case MCP::OperationMode::ByteMode16: // SEQOP = 1, BANK = 0
      bankMode_ = false;
      byteMode_ = true;
      break;

    case MCP::OperationMode::ByteMode8: // SEQOP = 1, BANK = 1
      bankMode_ = true;
      byteMode_ = true;
      break;
    }

    // Step 1: If switching to banked mode, write only BANK bit first
    if (bankMode_) {
      result |= i2cBus_.write_mcp_register(
          cntrlRegA->getAddress(), static_cast<uint8_t>(0x80), bankMode_);
      gpioBankA->updateBankMode(bankMode_);
      gpioBankB->updateBankMode(bankMode_);
    }

    // Step 2: Update remaining settings using configuration struct value
    uint8_t updatedSetting = configuration_.getSettingValue();
    cntrlRegA->configure<MCP::REG::IOCON>(updatedSetting);
    cntrlRegB->configure<MCP::REG::IOCON>(updatedSetting);

    if (bankMode_) {
      // In BANK mode (8-bit register mapping), write separately to each bank
      result |= i2cBus_.write_mcp_register(cntrlRegA->getAddress(),
                                           updatedSetting, bankMode_);
      result |= i2cBus_.write_mcp_register(cntrlRegB->getAddress(),
                                           updatedSetting, bankMode_);
    } else {
      // In 16-bit register mapping, write to both registers in sequence
      result |= i2cBus_.write_mcp_register(cntrlRegA->getAddress(),
                                           updatedSetting, bankMode_);
      result |= i2cBus_.write_mcp_register(cntrlRegA->getAddress() + 1,
                                           updatedSetting, bankMode_);
    }

    // Step 4: Update other MCP settings
    mirrorMode_ = (settings_.mirror == MCP::PairedInterrupt::Enabled);
    slewrateDisabled_ = (settings_.slew == MCP::Slew::Disabled);
    hardwareAddressing_ = (settings_.haen == MCP::HardwareAddr::Enabled);
    opendrainEnabled_ = (settings_.odr == MCP::OpenDrain::Enabled);
    interruptPolarityHigh_ =
        (settings_.intpol == MCP::InterruptPolarity::ActiveHigh);

    // Step 5: Handle result logging
    if (result != 0) {
      ESP_LOGE(MCP_TAG, "New settings change failed, reverting to defaults");
      configuration_.configureDefault();
    } else {
      ESP_LOGI(MCP_TAG, "Successfully changed the settings");
    }
  }
}

void MCPDevice::resetDevice() { ESP_LOGI(MCP_TAG, "resetting the device"); }
void MCPDevice::init() {

  i2cBus_.init();
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
void MCPDevice::digitalWrite(const int pin, const uint8_t level) {

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
  return gpioBank->setPinState(pinEnum, static_cast<bool>(level));
}
void MCPDevice::digitalWrite(const MCP::Pin pin, const uint8_t level) {

  uint8_t pinIndex = Util::getPinIndex(pin.getEnum());
  return digitalWrite(pinIndex, level);
}
void MCPDevice::digitalWrite(const MCP::PORT port, const uint8_t pinmask,
                             const uint8_t level) {

  GPIO_BANK *gpioBank =
      (port == MCP::PORT::GPIOA) ? gpioBankA.get() : gpioBankB.get();
  return gpioBank->setPinState(pinmask, static_cast<bool>(level));
}
void MCPDevice::digitalWrite(const MCP::PORT port, const uint8_t level) {
  GPIO_BANK *gpioBank =
      (port == MCP::PORT::GPIOA) ? gpioBankA.get() : gpioBankB.get();
  return gpioBank->setPinState(static_cast<bool>(level));
}

bool MCPDevice::digitalRead(const int pin) {
  MCP::PIN pinEnum;
  if (0 <= pin && pin <= 15) {
    pinEnum = static_cast<MCP::PIN>(pin);
  } else {
    assert(false && "Invalid pin");
  }
  MCP::PORT port = Util::getPortFromPin(pinEnum);

  GPIO_BANK *gpioBank =
      (port == MCP::PORT::GPIOA) ? gpioBankA.get() : gpioBankB.get();
  return gpioBank->getPinState(pinEnum);
}
bool MCPDevice::digitalRead(const MCP::Pin pin) {
  uint8_t pinIndex = Util::getPinIndex(pin.getEnum());
  return digitalRead(pinIndex);
}
uint8_t MCPDevice::digitalRead(const MCP::PORT port, const uint8_t pinmask) {
  GPIO_BANK *gpioBank =
      (port == MCP::PORT::GPIOA) ? gpioBankA.get() : gpioBankB.get();
  return gpioBank->getPinState(pinmask);
}
uint8_t MCPDevice::digitalRead(const MCP::PORT port) {
  GPIO_BANK *gpioBank =
      (port == MCP::PORT::GPIOA) ? gpioBankA.get() : gpioBankB.get();
  return gpioBank->getPinState();
}

void MCPDevice::invertInput(const int pin, bool invert) {

  MCP::PIN pinEnum;
  if (0 <= pin && pin <= 15) {
    pinEnum = static_cast<MCP::PIN>(pin);
  } else {
    assert(false && "Invalid pin");
  }
  MCP::PORT port = Util::getPortFromPin(pinEnum);

  GPIO_BANK *gpioBank =
      (port == MCP::PORT::GPIOA) ? gpioBankA.get() : gpioBankB.get();

  return gpioBank->setInputPolarity(invert == true
                                        ? MCP::INPUT_POLARITY::INVERTED
                                        : MCP::INPUT_POLARITY::UNCHANGED);
}

void MCPDevice::invertInput(const MCP::Pin pin, bool invert) {
  uint8_t pinIndex = Util::getPinIndex(pin.getEnum());
  return invertInput(pinIndex, invert);
}
void MCPDevice::invertInput(const MCP::PORT port, const uint8_t pinmask,
                            bool invert) {
  GPIO_BANK *gpioBank =
      (port == MCP::PORT::GPIOA) ? gpioBankA.get() : gpioBankB.get();
  return gpioBank->setInputPolarity(
      pinmask, invert == true ? MCP::INPUT_POLARITY::INVERTED
                              : MCP::INPUT_POLARITY::UNCHANGED);
}
void MCPDevice::invertInput(const MCP::PORT port, bool invert) {

  GPIO_BANK *gpioBank =
      (port == MCP::PORT::GPIOA) ? gpioBankA.get() : gpioBankB.get();
  return gpioBank->setInputPolarity(invert == true
                                        ? MCP::INPUT_POLARITY::INVERTED
                                        : MCP::INPUT_POLARITY::UNCHANGED);
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
  i2cBus_.write_mcp_register(regAddress, settings, bankMode_);

  Serial.printf("New Event %d BankMode changed \n", ev->id);
  EventManager::acknowledgeEvent(ev);
  EventManager::clearBits(RegisterEvent::BANK_MODE_CHANGED);
  xSemaphoreGive(regRWmutex);
}

void MCPDevice::handleReadEvent(currentEvent *ev) {
  MCP::REG reg = ev->regIdentity.reg;
  uint8_t currentAddress = ev->regIdentity.regAddress;
  MCP::PORT port = ev->regIdentity.port;
  uint8_t regAddress = 0;

  int value = i2cBus_.read_mcp_register(currentAddress, bankMode_);
  if (value == -1) {
    Serial.printf("Read failed for id=%d ; Invalid data received\n", ev->id);
    return;
  }

  if (!bankMode_) {
    if ((currentAddress % 2) != 0) {
      regAddress = currentAddress - 1; // Align with Port A
    }

    uint8_t valueA = value & 0xFF;
    uint8_t valueB = (value >> 8) & 0xFF;
    gpioBankA->updateRegisterValue(regAddress, valueA);
    gpioBankB->updateRegisterValue(regAddress + 1, valueB);

  } else {
    Serial.printf("Read event%d update for 8bit mode\n", ev->id);
    if (port == MCP::PORT::GPIOA) {
      gpioBankA->updateRegisterValue(regAddress, value);
    } else {
      gpioBankB->updateRegisterValue(regAddress, value);
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

  uint8_t result = i2cBus_.write_mcp_register(reg, value, bankMode_);

  if (result == 0) {
    ESP_LOGI(MCP_TAG, "New Write success for address 0x%02X for id=%d ; \n",
             reg, ev->id);
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
  uint8_t result = i2cBus_.write_mcp_register(reg, settings, bankMode_);
  if (result == 0) {
    Serial.printf("New Setting Event sucessfull id=%d ; \n", ev->id);
    EventManager::acknowledgeEvent(ev);
  } else {
    Serial.printf("New Setting Event failed for id=%d ; \n", ev->id);
  }

  EventManager::clearBits(RegisterEvent::SETTINGS_CHANGED);
  xSemaphoreGive(regRWmutex);
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
    ESP_LOGI(MCP_TAG, "Register: %s, Address: 0x%02X, Value: 0x%02X",
             Util::ToString::REG(reg), address, value);
  }

  // Dump PORTB Registers
  ESP_LOGI(MCP_TAG, "PORTB Registers:");
  for (uint8_t i = 0; i < MCP::MAX_REG_PER_PORT; ++i) {
    MCP::REG reg = static_cast<MCP::REG>(i);
    uint8_t address = getRegisterAddress(reg, MCP::PORT::GPIOB);
    uint8_t value = getRegisterSavedValue(reg, MCP::PORT::GPIOB);
    ESP_LOGI(MCP_TAG, "Register: %s, Address: 0x%02X, Value: 0x%02X",
             Util::ToString::REG(reg), address, value);
  }
}
void MCPDevice::updateRegisters(MCPDevice *device) {
  if (device) {
  }
}

} // namespace COMPONENT
