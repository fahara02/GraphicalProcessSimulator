#include "RegisterEvents.hpp"

EventGroupHandle_t EventManager::registerEventGroup = nullptr;
SemaphoreHandle_t EventManager::eventMutex = xSemaphoreCreateMutex();

std::array<currentEvent, EventManager::MAX_EVENTS> EventManager::eventBuffer;
std::atomic<size_t> EventManager::head{0};
std::atomic<size_t> EventManager::tail{0};
std::unordered_map<registerIdentity, size_t> EventManager::eventIndexMap;

const EventBits_t EventManager::REGISTER_EVENT_BITS_MASK =
    static_cast<EventBits_t>(RegisterEvent::BANK_MODE_CHANGED) |
    static_cast<EventBits_t>(RegisterEvent::READ_REQUEST) |
    static_cast<EventBits_t>(RegisterEvent::WRITE_REQUEST) |
    static_cast<EventBits_t>(RegisterEvent::SETTINGS_CHANGED) |
    static_cast<EventBits_t>(RegisterEvent::CLEAR_INTERRUPT) |
    static_cast<EventBits_t>(RegisterEvent::DATA_RECEIVED) |
    static_cast<EventBits_t>(RegisterEvent::RESTART) |
    static_cast<EventBits_t>(RegisterEvent::MAX);

void EventManager::initializeEventGroups() {
  if (registerEventGroup == nullptr) {
    registerEventGroup = xEventGroupCreate();
  }
}

void EventManager::setBits(RegisterEvent e) {
  xEventGroupSetBits(registerEventGroup, static_cast<EventBits_t>(e));
}

void EventManager::clearBits(RegisterEvent e) {
  xEventGroupClearBits(registerEventGroup, static_cast<EventBits_t>(e));
}

currentEvent *EventManager::getEvent(RegisterEvent eventType) {
  size_t currentHead = head.load(std::memory_order_acquire);
  size_t currentTail = tail.load(std::memory_order_acquire);

  while (currentHead != currentTail) {
    if (eventBuffer[currentHead].event == eventType &&
        !eventBuffer[currentHead].isAcknowledged()) {
      return &eventBuffer[currentHead];
    }
    currentHead = (currentHead + 1) % MAX_EVENTS;
  }
  return nullptr;
}

bool EventManager::acknowledgeEvent(currentEvent *event) {
  if (!event)
    return false;

  event->AcknowledgeEvent();
  eventIndexMap.erase(event->regIdentity);

  size_t currentHead = head.load(std::memory_order_relaxed);
  while (currentHead != tail.load(std::memory_order_acquire) &&
         eventBuffer[currentHead].isAcknowledged()) {
    head.store((currentHead + 1) % MAX_EVENTS, std::memory_order_release);
    currentHead = head.load(std::memory_order_relaxed);
  }
  return true;
}

bool EventManager::createEvent(registerIdentity identity, RegisterEvent e,
                               uint16_t valueOrSettings) {
  size_t currentTail = tail.load(std::memory_order_relaxed);
  size_t nextTail = (currentTail + 1) % MAX_EVENTS;

  if (nextTail == head.load(std::memory_order_acquire)) {
    return false; // Queue full
  }

  // if (eventIndexMap.find(identity) != eventIndexMap.end()) {
  //   return false;
  // }
  if (eventIndexMap.find(identity) != eventIndexMap.end()) {
    size_t index = eventIndexMap[identity];
    eventBuffer[index] = currentEvent(e, identity, valueOrSettings, index);
    setBits(e); // Ensure the event gets signaled
    return true;
  }

  eventBuffer[currentTail] =
      currentEvent(e, identity, valueOrSettings, currentTail);
  eventIndexMap[identity] = currentTail;

  tail.store(nextTail, std::memory_order_release);
  setBits(e);
  return true;
}

size_t EventManager::getQueueSize() {
  size_t h = head.load(std::memory_order_acquire);
  size_t t = tail.load(std::memory_order_acquire);
  return (t >= h) ? (t - h) : (MAX_EVENTS - h + t);
}
