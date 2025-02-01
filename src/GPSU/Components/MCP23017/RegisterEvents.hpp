#ifndef REGISTER_EVENTS_HPP
#define REGISTER_EVENTS_HPP
#include "Arduino.h"
#include "MCP_Constants.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "stdint.h"
#include <array>
#include <atomic>
#include <memory>

enum class RegisterEvent : EventBits_t {

  BANK_MODE_CHANGED = 1 << 0,
  READ_REQUEST = 1 << 1,
  WRITE_REQUEST = 1 << 2,
  SETTINGS_CHANGED = 1 << 3,
  CLEAR_INTERRUPT = 1 << 4,
  NETWORK_ERROR = 1 << 5,
  RESTART = 1 << 6,
  MAX = 1 << 7
};
struct registerIdentity {
  MCP::REG reg;
  MCP::PORT port;
  uint8_t regAddress;
  registerIdentity(MCP::REG r = MCP::REG::IODIR, MCP::PORT p = MCP::PORT::GPIOA,
                   uint8_t addr = 0xFF)
      : reg(r), port(p), regAddress(addr) {}

  registerIdentity(const registerIdentity &other)
      : reg(other.reg), port(other.port), regAddress(other.regAddress) {}

  bool operator==(const registerIdentity &other) const {
    return (reg == other.reg) && (port == other.port) &&
           (regAddress == other.regAddress);
  }

  bool operator!=(const registerIdentity &other) const {
    return !(*this == other);
  }
};

struct currentEvent {

  const RegisterEvent event;
  const registerIdentity regIdentity;
  const uint16_t data;
  const int id;

  currentEvent(RegisterEvent e = RegisterEvent::MAX,
               registerIdentity identity = registerIdentity{},
               uint16_t valueOrSettings = 0, uint8_t _id = 0)
      : event(e), regIdentity(identity), data(valueOrSettings), id(_id),
        acknowledged_(false){

        };

  bool isIdentical(const RegisterEvent &e, const registerIdentity &identity,
                   uint8_t valueOrSettings) const {
    return (event == e) && (regIdentity == identity) &&
           (data == valueOrSettings);
  }
  void AcknowledgeEvent() {
    if (!isAckKnowledged()) {
      acknowledged_ = true;
    }
  }
  bool isAckKnowledged() const { return acknowledged_; }

private:
  bool acknowledged_;
};
class EventManager {
public:
  static EventGroupHandle_t registerEventGroup;

  static void setBits(RegisterEvent e);
  static void clearBits(RegisterEvent e);
  static void initializeEventGroups();
  static bool createEvent(registerIdentity identity, RegisterEvent e,
                          uint16_t valueOrSettings = 0);
  static currentEvent *getEvent(RegisterEvent eventType);
  static bool acknowledgeEvent(currentEvent *ev);
  static size_t getQueueSize() { return event_queue.getQueueSize(); }
  static bool clearOldestEvent() { return event_queue.removeOldestEvent(); }

protected:
  struct eventQueue {
    static std::array<std::unique_ptr<currentEvent>, MCP::MAX_EVENT>
        unresolvedEvents;
    static std::array<size_t, static_cast<size_t>(RegisterEvent::MAX)>
        unresolvedEventCounts;
    static size_t head; // Points to the oldest event
    static size_t tail; // Points to the next free slot
    bool isEmpty() { return head == tail; }
    bool isFull() { return (tail + 1) % MCP::MAX_EVENT == head; }
    size_t advance(size_t index) const { return (index + 1) % MCP::MAX_EVENT; }

    currentEvent *getNextEvent(size_t currentIndex) {
      size_t next_index = advance(currentIndex);
      return unresolvedEvents[next_index].get();
    }

    bool insertEvent(std::unique_ptr<currentEvent> ev) {
      if (isFull()) {
        return false;
      }
      int id = ev->id;
      // Check for duplicates and find a cleared slot (nullptr)
      size_t currentIndex = head;
      while (currentIndex != tail) {
        currentEvent *event = unresolvedEvents[currentIndex].get();
        if (event && event->isIdentical(ev->event, ev->regIdentity, ev->data)) {
          return false; // Duplicate found
        }
        if (!event) {
          // Reuse cleared slot
          unresolvedEvents[currentIndex] = std::move(ev);
          unresolvedEventCounts[static_cast<size_t>(ev->event)]++;
          return true;
        }
        currentIndex = advance(currentIndex);
      }
      // If no cleared slot is found, insert at the tail
      unresolvedEvents[tail] = std::move(ev);
      ESP_LOGI("EVENT_MANAGER", "created event for position =%d for id=%d",
               tail, id);
      unresolvedEventCounts[static_cast<size_t>(
          unresolvedEvents[tail]->event)]++;
      tail = advance(tail);

      return true;
    }

    void decrementEventCount(RegisterEvent eventType) {
      unresolvedEventCounts[static_cast<size_t>(eventType)]--;
    }
    void markEventResolved(size_t index) {
      unresolvedEvents[index].get()->AcknowledgeEvent();
    }

    void removeResolvedEvents() {
      while (head != tail && unresolvedEvents[head]->isAckKnowledged()) {
        decrementEventCount(unresolvedEvents[head]->event);
        unresolvedEvents[head].reset();
        head = advance(head);
      }
    }
    currentEvent *getOldestEvent(RegisterEvent eventType) const {
      size_t currentIndex = head;
      while (currentIndex != tail) {
        currentEvent *event = unresolvedEvents[currentIndex].get();

        if (event && event->event == eventType && !event->isAckKnowledged()) {
          return event;
        }
        currentIndex = advance(currentIndex);
      }
      return nullptr;
    }
    bool removeOldestEvent() {
      if (isEmpty()) {
        return false; // Queue is empty, nothing to remove
      }

      if (unresolvedEvents[head]) {
        decrementEventCount(unresolvedEvents[head]->event);
        unresolvedEvents[head].reset(); // Release the unique_ptr
        head = advance(head);
        return true;
      }
      return false;
    }
    size_t getQueueSize() const {
      return (tail >= head) ? (tail - head) : (MCP::MAX_EVENT - head + tail);
    }
  };

private:
  static const EventBits_t REGISTER_EVENT_BITS_MASK;
  static SemaphoreHandle_t eventMutex;

  static registerIdentity latest_identity;
  static currentEvent current_event;
  static std::atomic<uint8_t> event_counter;
  static eventQueue event_queue;

  static int findOldestEventIndex(RegisterEvent eventType);
  static int findFreeSlot();
  static int getNextId();
  static void cleanupEventGroups();
};

#endif