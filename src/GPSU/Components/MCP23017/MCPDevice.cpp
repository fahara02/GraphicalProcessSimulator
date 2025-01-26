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

void MCPDevice::startEventMonitorTask(MCPDevice *device) {
  xTaskCreate(EventMonitorTask, "EventMonitorTask", 4096, device, 5, nullptr);
}

void MCPDevice::init() {

  bankMode_ = cntrlRegA->getBankMode<MCP::REG::IOCON>() ||
              cntrlRegB->getBankMode<MCP::REG::IOCON>();
  startEventMonitorTask(this);
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
        ESP_LOGI(MCP_TAG, "Read Register 0x%02X: Value = 0x%02X", reg, value);
        xSemaphoreGive(regRWmutex);
      }
    }

    // Handle WRITE_REQUEST
    if (events & static_cast<EventBits_t>(RegisterEvent::WRITE_REQUEST)) {
      if (xSemaphoreTake(regRWmutex, portMAX_DELAY)) {
        uint8_t reg = EventManager::getCurrentEvent().regAddress;
        uint8_t value = EventManager::getCurrentEvent().value;
        device->write_mcp_register(reg, value);
        ESP_LOGI(MCP_TAG, "Wrote Value 0x%02X to Register 0x%02X", value, reg);
        xSemaphoreGive(regRWmutex);
      }
    }

    // Handle BANK_MODE_CHANGED
    if (events & static_cast<EventBits_t>(RegisterEvent::BANK_MODE_CHANGED)) {
      if (xSemaphoreTake(regRWmutex, portMAX_DELAY)) {
        MCP::PORT port = EventManager::getCurrentEvent().port;
        uint8_t regAddress = EventManager::getCurrentEvent().regAddress;

        // Get saved settings for the port
        uint8_t settings = EventManager::getCurrentEvent().settings;

        // Write the settings to the register
        device->write_mcp_register(regAddress, settings);

        ESP_LOGI(
            MCP_TAG,
            "Bank mode changed. Port: %d, Register: 0x%02X, Settings: 0x%02X",
            static_cast<int>(port), regAddress, settings);
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

        ESP_LOGI(
            MCP_TAG,
            "Settings changed. Port: %d, Register: 0x%02X, Settings: 0x%02X",
            static_cast<int>(port), regAddress, settings);
        xSemaphoreGive(regRWmutex);
      }
    }
  }
}

} // namespace COMPONENT
