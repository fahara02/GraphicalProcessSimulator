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
      intA_(GPIO_NUM_NC), intB_(GPIO_NUM_NC),
      i2cBus_(MCP::I2CBus::getInstance(address_, sda_, scl_)),
      //
      cntrlRegA(std::make_shared<MCP::Register>(model_, MCP::REG::IOCON,
                                                MCP::PORT::GPIOA, bankMode_)),
      cntrlRegB(std::make_shared<MCP::Register>(model_, MCP::REG::IOCON,
                                                MCP::PORT::GPIOB, bankMode_)),
      gpioBankA(
          std::make_unique<MCP::GPIO_BANK>(MCP::PORT::GPIOA, model, cntrlRegA)),
      gpioBankB(
          std::make_unique<MCP::GPIO_BANK>(MCP::PORT::GPIOB, model, cntrlRegB)),

      interruptManager_(std::make_unique<MCP::InterruptManager>(
          model, i2cBus_, cntrlRegA, cntrlRegB))

{
  init();
  loadSettings();
}
MCPDevice::~MCPDevice() = default;
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
  // gpioBankB->setInterruptMask(0xFF);
  interruptManager_->setupIntteruptMask(0X00, 0XFF);
  setupIntterupt();
}
bool MCPDevice::enableInterrupt() {
  return interruptManager_->enableInterrupt();
}
void MCPDevice::startEventMonitorTask(MCPDevice *device) {
  if (!device) {
    ESP_LOGE(MCP_TAG, "no_device");
  } else {
    xTaskCreatePinnedToCore(EventMonitorTask, "EventMonitorTask", 8192, device,
                            5, &eventTaskHandle, 0);
  }
}

void MCPDevice::pinMode(const MCP::PORT port, const uint8_t pinmask,
                        const uint8_t mode) {
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
void MCPDevice::pinMode(const MCP::PORT port, const uint8_t mode) {
  GPIO_BANK *gpioBank =
      (port == MCP::PORT::GPIOA) ? gpioBankA.get() : gpioBankB.get();
  return pinMode(port, gpioBank->getGeneralMask(), mode);
}

void MCPDevice::pinMode(const MCP::Pin pin, const uint8_t mode) {
  pinMode(Util::getPortFromPin(pin.getEnum()), pin.getMask(), mode);
}

void MCPDevice::pinMode(const int pin, const uint8_t mode) {
  assert(pin >= 0 && pin <= 15 && "Invalid pin");
  MCP::PIN pinEnum = static_cast<MCP::PIN>(pin);
  MCP::PORT port = Util::getPortFromPin(pinEnum);
  uint8_t mask = 1 << static_cast<uint8_t>(pinEnum) % 8;
  pinMode(port, mask, mode);
}

void MCPDevice::digitalWrite(const MCP::PORT port, const uint8_t pinmask,
                             const uint8_t level) {
  GPIO_BANK *gpioBank =
      (port == MCP::PORT::GPIOA) ? gpioBankA.get() : gpioBankB.get();
  gpioBank->setPinState(pinmask, static_cast<bool>(level));
}
void MCPDevice::digitalWrite(const MCP::PORT port, const uint8_t level) {
  GPIO_BANK *gpioBank =
      (port == MCP::PORT::GPIOA) ? gpioBankA.get() : gpioBankB.get();
  gpioBank->setPinState(static_cast<bool>(level));
}
void MCPDevice::digitalWrite(const MCP::Pin pin, const uint8_t level) {
  digitalWrite(Util::getPortFromPin(pin.getEnum()), pin.getMask(), level);
}

void MCPDevice::digitalWrite(const int pin, const uint8_t level) {
  assert(pin >= 0 && pin <= 15 && "Invalid pin");
  MCP::PIN pinEnum = static_cast<MCP::PIN>(pin);
  MCP::PORT port = Util::getPortFromPin(pinEnum);
  uint8_t mask = 1 << static_cast<uint8_t>(pinEnum) % 8;
  digitalWrite(port, mask, level);
}

uint8_t MCPDevice::digitalRead(const MCP::PORT port, const uint8_t pinmask) {
  GPIO_BANK *gpioBank =
      (port == MCP::PORT::GPIOA) ? gpioBankA.get() : gpioBankB.get();
  return gpioBank->getPinState(pinmask);
}
bool MCPDevice::digitalRead(const MCP::Pin pin) {
  return digitalRead(Util::getPortFromPin(pin.getEnum()), pin.getMask());
}
uint8_t MCPDevice::digitalRead(const MCP::PORT port) {
  GPIO_BANK *gpioBank =
      (port == MCP::PORT::GPIOA) ? gpioBankA.get() : gpioBankB.get();
  return gpioBank->getPinState();
}

bool MCPDevice::digitalRead(const int pin) {
  assert(pin >= 0 && pin <= 15 && "Invalid pin");
  MCP::PIN pinEnum = static_cast<MCP::PIN>(pin);
  MCP::PORT port = Util::getPortFromPin(pinEnum);
  uint8_t mask = 1 << static_cast<uint8_t>(pinEnum) % 8;
  return digitalRead(port, mask);
}

void MCPDevice::invertInput(const MCP::PORT port, const uint8_t pinmask,
                            bool invert) {
  GPIO_BANK *gpioBank =
      (port == MCP::PORT::GPIOA) ? gpioBankA.get() : gpioBankB.get();
  gpioBank->setInputPolarity(pinmask, invert ? MCP::INPUT_POLARITY::INVERTED
                                             : MCP::INPUT_POLARITY::UNCHANGED);
}
void MCPDevice::invertInput(const MCP::PORT port, bool invert) {
  GPIO_BANK *gpioBank =
      (port == MCP::PORT::GPIOA) ? gpioBankA.get() : gpioBankB.get();
  gpioBank->setInputPolarity(invert ? MCP::INPUT_POLARITY::INVERTED
                                    : MCP::INPUT_POLARITY::UNCHANGED);
}

void MCPDevice::invertInput(const MCP::Pin pin, bool invert) {
  invertInput(Util::getPortFromPin(pin.getEnum()), pin.getMask(), invert);
}

void MCPDevice::invertInput(int pin, bool invert) {
  assert(pin >= 0 && pin <= 15 && "Invalid pin");
  MCP::PIN pinEnum = static_cast<MCP::PIN>(pin);
  MCP::PORT port = Util::getPortFromPin(pinEnum);
  uint8_t mask = 1 << static_cast<uint8_t>(pinEnum) % 8;
  invertInput(port, mask, invert);
}

int MCPDevice::readRegister(MCP::PORT port, MCP::REG regType) const {
  GPIO_BANK *gpioBank =
      (port == MCP::PORT::GPIOA) ? gpioBankA.get() : gpioBankB.get();
  return i2cBus_.read_mcp_register(gpioBank->getAddress(regType), bankMode_);
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
    ESP_LOGE(MCP_TAG, "Read failed for id=%d ; Invalid data received\n",
             ev->id);
    return;
  }

  if (!bankMode_) {
    if ((currentAddress % 2) != 0) {
      regAddress = currentAddress - 1; // Align with Port A
    } else {
      regAddress = currentAddress;
    }

    uint8_t valueA = static_cast<uint16_t>(value) & 0xFF;
    uint8_t valueB = (static_cast<uint16_t>(value) >> 8) & 0xFF;

    gpioBankA->updateRegisterValue(regAddress, valueA);
    gpioBankB->updateRegisterValue(regAddress + 1, valueB);

  } else {

    if (port == MCP::PORT::GPIOA) {
      gpioBankA->updateRegisterValue(currentAddress,
                                     static_cast<uint8_t>(value));
    } else {
      gpioBankB->updateRegisterValue(currentAddress,
                                     static_cast<uint8_t>(value));
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

MCP::Register *MCPDevice::getRegister(MCP::REG reg, MCP::PORT port) {
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
  bool is16Bit = !bankMode_; // True if in 16-bit mode

  if (is16Bit) {
    ESP_LOGI(MCP_TAG,
             "Register mapping: 16-bit mode (PORTA & PORTB separated)");

    for (uint8_t i = 0; i < MCP::MAX_REG_PER_PORT; i++) {
      MCP::REG regA = static_cast<MCP::REG>(i);
      MCP::REG regB = static_cast<MCP::REG>(i);

      uint8_t addressA = getRegisterAddress(regA, MCP::PORT::GPIOA);
      int combinedValue = i2cBus_.read_mcp_register(addressA, bankMode_);

      uint8_t lowByte = combinedValue & 0xFF;  // Extract PORTA value
      uint8_t highByte = (combinedValue >> 8); // Extract PORTB value

      ESP_LOGI(MCP_TAG, "Register: %s (PORTA), Address: 0x%02X, Value: 0x%02X",
               Util::ToString::REG(regA), addressA, lowByte);

      ESP_LOGI(MCP_TAG, "Register: %s (PORTB), Address: 0x%02X, Value: 0x%02X",
               Util::ToString::REG(regB), addressA + 1, highByte);
    }
  } else {
    ESP_LOGI(MCP_TAG, "Register mapping: 8-bit mode");

    // Read PORTA Registers
    ESP_LOGI(MCP_TAG, "PORTA:");
    for (uint8_t i = 0; i < MCP::MAX_REG_PER_PORT; i++) {
      MCP::REG reg = static_cast<MCP::REG>(i);
      uint8_t address = getRegisterAddress(reg, MCP::PORT::GPIOA);
      uint8_t value = i2cBus_.read_mcp_register(address, bankMode_);
      ESP_LOGI(MCP_TAG, "Register: %s, Address: 0x%02X, Value: 0x%02X",
               Util::ToString::REG(reg), address, value);
    }

    // Read PORTB Registers
    ESP_LOGI(MCP_TAG, "PORTB:");
    for (uint8_t i = 0; i < MCP::MAX_REG_PER_PORT; i++) {
      MCP::REG reg = static_cast<MCP::REG>(i);
      uint8_t address = getRegisterAddress(reg, MCP::PORT::GPIOB);
      uint8_t value = i2cBus_.read_mcp_register(address, bankMode_);
      ESP_LOGI(MCP_TAG, "Register: %s, Address: 0x%02X, Value: 0x%02X",
               Util::ToString::REG(reg), address, value);
    }
  }
}

void MCPDevice::setupIntterupt(MCP::INTR_TYPE type,
                               MCP::INTR_OUTPUT_TYPE outtype,
                               MCP::PairedInterrupt sharedIntr) {
  interruptManager_->setup(static_cast<int>(intA_), static_cast<int>(intB_),
                           type, outtype, sharedIntr);
}

} // namespace COMPONENT
