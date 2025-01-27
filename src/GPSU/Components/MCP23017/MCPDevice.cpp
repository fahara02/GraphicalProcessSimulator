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
                            1, &eventTaskHandle, 0);
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

    const EventBits_t CHECK_BITS_MASK =
        static_cast<EventBits_t>(RegisterEvent::READ_REQUEST) |
        static_cast<EventBits_t>(RegisterEvent::WRITE_REQUEST) |
        static_cast<EventBits_t>(RegisterEvent::BANK_MODE_CHANGED) |
        static_cast<EventBits_t>(RegisterEvent::SETTINGS_CHANGED);
    EventBits_t events =
        xEventGroupWaitBits(EventManager::registerEventGroup, CHECK_BITS_MASK,

                            pdTRUE,  // Clear the bits after processing
                            pdFALSE, // Wait for any event
                            portMAX_DELAY);

    // Handle READ_REQUEST
    if (events & static_cast<EventBits_t>(RegisterEvent::READ_REQUEST)) {
      if (xSemaphoreTake(regRWmutex, portMAX_DELAY)) {
        uint8_t reg = EventManager::getCurrentEvent().regAddress;
        uint8_t value = device->read_mcp_register(reg);
        xSemaphoreGive(regRWmutex);
      }
    }

    // Handle WRITE_REQUEST
    if (events & static_cast<EventBits_t>(RegisterEvent::WRITE_REQUEST)) {
      if (xSemaphoreTake(regRWmutex, portMAX_DELAY)) {
        uint8_t reg = EventManager::getCurrentEvent().regAddress;
        uint8_t value = EventManager::getCurrentEvent().value;
        device->write_mcp_register(reg, value);
        xSemaphoreGive(regRWmutex);
      }
    }

    // Handle BANK_MODE_CHANGED
    if (events & static_cast<EventBits_t>(RegisterEvent::BANK_MODE_CHANGED)) {
      if (xSemaphoreTake(regRWmutex, portMAX_DELAY)) {
        MCP::PORT port = EventManager::getCurrentEvent().port;
        uint8_t regAddress = EventManager::getCurrentEvent().regAddress;
        uint8_t settings = EventManager::getCurrentEvent().settings;
        device->write_mcp_register(regAddress, settings);
        xSemaphoreGive(regRWmutex);
      }
    }

    // Handle SETTINGS_CHANGED
    if (events & static_cast<EventBits_t>(RegisterEvent::SETTINGS_CHANGED)) {
      if (xSemaphoreTake(regRWmutex, portMAX_DELAY)) {
        MCP::PORT port = EventManager::getCurrentEvent().port;
        uint8_t regAddress = EventManager::getCurrentEvent().regAddress;
        uint8_t settings = EventManager::getCurrentEvent().settings;
        device->write_mcp_register(regAddress, settings);

        xSemaphoreGive(regRWmutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
  vTaskDelete(NULL);
}

} // namespace COMPONENT
