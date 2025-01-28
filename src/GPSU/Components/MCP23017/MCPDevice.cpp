#include "MCPDevice.hpp"
using namespace MCP;
namespace COMPONENT {
SemaphoreHandle_t MCPDevice::regRWmutex = xSemaphoreCreateMutex();
MCPDevice::MCPDevice(uint8_t address, MCP::MCP_MODEL model)
    : model_(model), address_(address),    //
      sda_(GPIO_NUM_21),                   //
      scl_(GPIO_NUM_22),                   //
      cs_(GPIO_NUM_NC),                    //
      reset_(GPIO_NUM_33),                 //
      wire_(std::make_unique<TwoWire>(0)), //
      gpioBankA(std::make_unique<MCP::GPIO_BANK>(MCP::PORT::GPIOA, model)),
      gpioBankB(std::make_unique<MCP::GPIO_BANK>(MCP::PORT::GPIOB, model)),
      cntrlRegA(gpioBankA->getControlRegister()),
      cntrlRegB(gpioBankB->getControlRegister())

{
  init();
}

void MCPDevice::init() {
  wire_->begin(address_, sda_, scl_, DEFAULT_I2C_CLK_FRQ);
  EventManager::initializeEventGroups();
  bankMode_ = cntrlRegA->getBankMode<MCP::REG::IOCON>() ||
              cntrlRegB->getBankMode<MCP::REG::IOCON>();
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
uint8_t MCPDevice::read_mcp_register(const uint8_t reg) {
  wire_->beginTransmission(address_);
  wire_->write(reg);
  wire_->endTransmission(false);
  wire_->requestFrom((uint8_t)address_, (uint8_t)1, (uint8_t) true);
  while (wire_->available() == 0)
    ;

  return wire_->read();
}
void MCPDevice::write_mcp_register(const uint8_t reg, uint8_t value) {
  wire_->beginTransmission(address_);
  wire_->write(reg);
  wire_->write(value);
  wire_->endTransmission(true);
}

void MCPDevice::EventMonitorTask(void *param) {
  MCPDevice *device = static_cast<MCPDevice *>(param);

  while (true) {
    // Wait for events
    size_t queueSize = EventManager::getQueueSize();
    if (MAX_EVENT - queueSize < 2) {

      if (xSemaphoreTake(regRWmutex, MUTEX_TIMEOUT)) {
        EventManager::clearOldestEvent();
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
  // uint8_t value = read_mcp_register(reg);

  Serial.printf("New ReadEvent id=%d ;\n", ev->id);
  EventManager::acknowledgeEvent(ev);
  EventManager::clearBits(RegisterEvent::READ_REQUEST);
  xSemaphoreGive(regRWmutex);
}
void MCPDevice::handleWriteEvent(currentEvent *ev) {
  uint8_t reg = ev->regIdentity.regAddress;
  uint8_t value = ev->data;

  write_mcp_register(reg, value);

  Serial.printf("New WriteEvent id=%d ; \n", ev->id);
  EventManager::acknowledgeEvent(ev);
  EventManager::clearBits(RegisterEvent::WRITE_REQUEST);
  xSemaphoreGive(regRWmutex);
}
void MCPDevice::handleSettingChangeEvent(currentEvent *ev) {
  MCP::PORT port = ev->regIdentity.port;
  uint8_t regAddress = ev->regIdentity.regAddress;
  uint8_t settings = ev->data;
  write_mcp_register(regAddress, settings);
  Serial.printf("New SettingChangeEvent id=%d ; \n", ev->id);
  EventManager::acknowledgeEvent(ev);
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

} // namespace COMPONENT
