#include "RegisterEvents.hpp"

EventGroupHandle_t EventManager::registerEventGroup = nullptr;
SemaphoreHandle_t EventManager::eventMutex = xSemaphoreCreateMutex();
currentEvent EventManager::current_event = {};
const EventBits_t EventManager::REGISTER_EVENT_BITS_MASK =
    static_cast<EventBits_t>(RegisterEvent::NONE) |
    static_cast<EventBits_t>(RegisterEvent::BANK_MODE_CHANGED) |
    static_cast<EventBits_t>(RegisterEvent::READ_REQUEST) |
    static_cast<EventBits_t>(RegisterEvent::WRITE_REQUEST) |
    static_cast<EventBits_t>(RegisterEvent::SETTINGS_CHANGED) |
    static_cast<EventBits_t>(RegisterEvent::CLEAR_INTERRUPT) |
    static_cast<EventBits_t>(RegisterEvent::NETWORK_ERROR) |
    static_cast<EventBits_t>(RegisterEvent::RESTART);

void EventManager::initializeEventGroups() {

  if (registerEventGroup == nullptr)
    registerEventGroup = xEventGroupCreate();
}

void EventManager::createEvent(uint8_t addr, RegisterEvent e, uint8_t value,
                               uint8_t settings) {
  if (xSemaphoreTake(eventMutex, portMAX_DELAY) == pdTRUE) {
    current_event.regAddress = addr;
    current_event.event = e;
    current_event.value = value;
    current_event.settings = settings;

    setBits(e); // Signal the event
    xSemaphoreGive(eventMutex);
  }
}

void EventManager::cleanupEventGroups() {
  if (registerEventGroup != nullptr) {
    vEventGroupDelete(registerEventGroup);
    registerEventGroup = nullptr;
  }
}

void EventManager::setBits(RegisterEvent e) {
  xEventGroupSetBits(registerEventGroup, static_cast<EventBits_t>(e));
}
void EventManager::clearBits(RegisterEvent e) {
  xEventGroupClearBits(registerEventGroup, static_cast<EventBits_t>(e));
}

currentEvent &EventManager::getCurrentEvent() { return current_event; }