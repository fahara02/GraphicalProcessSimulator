#include "RegisterEvents.hpp"
static int resolved_req = 0;
EventGroupHandle_t EventManager::registerEventGroup = nullptr;
SemaphoreHandle_t EventManager::eventMutex = xSemaphoreCreateMutex();
currentEvent EventManager::current_event = {};
std::array<std::unique_ptr<currentEvent>, MCP::MAX_EVENT>
    EventManager::eventQueue::unresolvedEvents = {nullptr};
std::array<size_t, static_cast<size_t>(RegisterEvent::MAX)>
    EventManager::eventQueue::unresolvedEventCounts = {};
size_t EventManager::eventQueue::head = 0;
size_t EventManager::eventQueue::tail = 0;
std::atomic<uint8_t> EventManager::event_counter{0};

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

int EventManager::getNextId() {
  int currentId = event_counter.fetch_add(1);
  if (currentId >= MCP::MAX_EVENT) {
    event_counter.store(0); // Reset counter if it exceeds MAX_EVENT
    currentId = 0;          // Start from 0 again
  }
  return currentId;
}

currentEvent *EventManager::getEvent(RegisterEvent eventType) {
  if (xSemaphoreTake(eventMutex, MCP::RW_MUTEX_TIMEOUT) != pdTRUE) {
    ESP_LOGE("EVENT_MANAGER", "Failed to acquire semaphore!");
    return nullptr;
  }
  currentEvent *oldestEvent = event_queue.getOldestEvent(eventType);
  xSemaphoreGive(eventMutex);
  return oldestEvent;
}

bool EventManager::acknowledgeEvent(currentEvent *event) {

  if (xSemaphoreTake(eventMutex, MCP::RW_MUTEX_TIMEOUT) != pdTRUE) {
    ESP_LOGE("EVENT_MANAGER", "Failed to acquire semaphore!");
    xSemaphoreGive(eventMutex);
    return false;
  }

  // size_t currentIndex = event_queue.head;
  // while (currentIndex != event_queue.tail) {
  //   currentEvent *event = event_queue.unresolvedEvents[currentIndex].get();
  //   Serial.printf("matching id ...%d", eventId);
  //   if (event && event->id == eventId && !event->isAckKnowledged()) {
  //     event->AcknowledgeEvent();
  //     event_queue.removeResolvedEvents();

  //     xSemaphoreGive(eventMutex);
  //     return true;
  //   }

  //   currentIndex = event_queue.advance(currentIndex);
  // }
  event->AcknowledgeEvent();
  event_queue.removeResolvedEvents();
  resolved_req += 1;
  Serial.printf("acknowdged %d event resolved request Total %d\n", event->id,
                resolved_req);
  Serial.println("");

  xSemaphoreGive(eventMutex);
  return false;
}

bool EventManager::createEvent(registerIdentity identity, RegisterEvent e,
                               uint8_t valueOrSettings) {
  if (xSemaphoreTake(eventMutex, MCP::RW_MUTEX_TIMEOUT) != pdTRUE) {
    ESP_LOGE("EVENT_MANAGER", "Failed to acquire semaphore!");
    return false;
  }

  std::unique_ptr<currentEvent> newEvent =
      std::make_unique<currentEvent>(e, identity, valueOrSettings, getNextId());

  bool inserted = event_queue.insertEvent(std::move(newEvent));
  if (inserted) {
    setBits(e);
  }
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