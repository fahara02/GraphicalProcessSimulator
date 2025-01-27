#include "RegisterEvents.hpp"

EventGroupHandle_t EventManager::registerEventGroup = nullptr;
SemaphoreHandle_t EventManager::eventMutex = xSemaphoreCreateMutex();
currentEvent EventManager::current_event = {};
std::array<currentEvent, MCP::MAX_EVENT>
    EventManager::eventQueue::unresolvedEvents = {};
std::array<size_t, static_cast<size_t>(RegisterEvent::MAX)>
    EventManager::eventQueue::unresolvedEventCounts = {};
size_t EventManager::eventQueue::head = 0;
size_t EventManager::eventQueue::tail = 0;
uint8_t EventManager::event_counter = 0;

const EventBits_t EventManager::REGISTER_EVENT_BITS_MASK =

    static_cast<EventBits_t>(RegisterEvent::BANK_MODE_CHANGED) |
    static_cast<EventBits_t>(RegisterEvent::READ_REQUEST) |
    static_cast<EventBits_t>(RegisterEvent::WRITE_REQUEST) |
    static_cast<EventBits_t>(RegisterEvent::SETTINGS_CHANGED) |
    static_cast<EventBits_t>(RegisterEvent::CLEAR_INTERRUPT) |
    static_cast<EventBits_t>(RegisterEvent::NETWORK_ERROR) |
    static_cast<EventBits_t>(RegisterEvent::RESTART) |
    static_cast<EventBits_t>(RegisterEvent::MAX);

void EventManager::initializeEventGroups() {

  if (registerEventGroup == nullptr)
    registerEventGroup = xEventGroupCreate();
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

int EventManager::getNextId() {
  if (event_counter < MCP::MAX_EVENT) {
    event_counter += 1;
  } else {
    event_counter = 0;
  }
  return event_counter;
}

currentEvent EventManager::getEvent(RegisterEvent eventType) {
  if (xSemaphoreTake(eventMutex, portMAX_DELAY) != pdTRUE) {
    return currentEvent();
  }
  currentEvent oldestEvent = event_queue.getOldestEvent(eventType);
  xSemaphoreGive(eventMutex);
  return oldestEvent;
}

bool EventManager::acknowledgeEvent(int eventId) {
  if (xSemaphoreTake(eventMutex, portMAX_DELAY) != pdTRUE) {
    return false;
  }

  size_t currentIndex = event_queue.head;
  while (currentIndex != event_queue.tail) {
    auto &event = event_queue.unresolvedEvents[currentIndex];

    if (event.getId() == eventId && !event.isAckKnowledged()) {
      event.AcknowledgeEvent();
      event_queue.removeResolvedEvents();

      xSemaphoreGive(eventMutex);
      return true;
    }

    currentIndex = event_queue.advance(currentIndex);
  }

  xSemaphoreGive(eventMutex);
  return false;
}

bool EventManager::createEvent(registerIdentity identity, RegisterEvent e,
                               uint8_t valueOrSettings) {
  if (xSemaphoreTake(eventMutex, portMAX_DELAY) != pdTRUE) {
    return false;
  }

  // Check for duplicates
  size_t currentIndex = event_queue.head;
  while (currentIndex != event_queue.tail) {
    const auto &event = event_queue.getNextEvent(currentIndex);
    if (event.isIdentical(e, identity, valueOrSettings)) {
      xSemaphoreGive(eventMutex);
      return false; // Duplicate found
    }
    currentIndex = event_queue.advance(currentIndex);
  }

  // Create and insert the new event
  currentEvent newEvent(e, identity, valueOrSettings, getNextId());
  bool inserted = event_queue.insertEvent(newEvent);

  xSemaphoreGive(eventMutex);
  return inserted;
}

// int EventManager::findFreeSlot() {
//   for (size_t i = 0; i < unresolvedEvents.size(); ++i) {
//     if (unresolvedEvents[i].getId() == -1) {
//       return static_cast<int>(i);
//     }
//   }
//   return -1;
// }

// int EventManager::findOldestEventIndex(RegisterEvent eventType) {

//   std::array<int, MAX_EVENT> matchingIndices = {-1};
//   size_t matchCount = 0;

//   for (size_t i = 0; i < unresolvedEvents.size(); ++i) {
//     const auto &event = unresolvedEvents[i];
//     if (event.event == eventType && !event.isAckKnowledged()) {
//       matchingIndices[matchCount++] = static_cast<int>(i);
//     }
//   }

//   if (matchCount == 0) {
//     return -1;
//   }

//   std::sort(matchingIndices.begin(), matchingIndices.begin() + matchCount,
//             [&](int a, int b) {
//               return unresolvedEvents[a].getId() <
//               unresolvedEvents[b].getId();
//             });

//   return matchingIndices[0];
// }