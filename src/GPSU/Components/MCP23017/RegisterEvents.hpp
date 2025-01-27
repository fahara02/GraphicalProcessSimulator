#ifndef REGISTER_EVENTS_HPP
#define REGISTER_EVENTS_HPP
#include "MCP_Constants.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "stdint.h"
#include <array>

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

  RegisterEvent event;
  registerIdentity regIdentity;
  uint8_t data;

  currentEvent(RegisterEvent e = RegisterEvent::MAX,
               registerIdentity identity = registerIdentity{},
               uint8_t valueOrSettings = 0, uint8_t id = -1)
      : event(e), regIdentity(identity), data(valueOrSettings), id_(id),
        acknowledged_(false){

        };
  constexpr currentEvent &operator=(const currentEvent &other) {
    if (this != &other) {

      event = other.event;
      regIdentity = other.regIdentity;
      data = other.data;
      id_ = other.getId();
    }
    return *this;
  }
  bool isIdentical(const RegisterEvent &e, const registerIdentity &identity,
                   uint8_t valueOrSettings) const {
    return (event == e) && (regIdentity == identity) &&
           (data == valueOrSettings);
  }
  void AcknowledgeEvent() { acknowledged_ = true; }
  bool isAckKnowledged() const { return acknowledged_; }
  int getId() const { return id_; }

private:
  int id_;
  bool acknowledged_;
};
class EventManager {
public:
  static EventGroupHandle_t registerEventGroup;

  static void setBits(RegisterEvent e);
  static void clearBits(RegisterEvent e);
  static void initializeEventGroups();
  static bool createEvent(registerIdentity identity, RegisterEvent e,
                          uint8_t valueOrSettings = 0);
  static currentEvent getEvent(RegisterEvent eventType);
  static bool acknowledgeEvent(int eventId);
  static currentEvent &getCurrentEvent();

protected:
  struct eventQueue {
    static std::array<currentEvent, MCP::MAX_EVENT> unresolvedEvents;
    static std::array<size_t, static_cast<size_t>(RegisterEvent::MAX)>
        unresolvedEventCounts;
    static size_t head; // Points to the oldest event
    static size_t tail; // Points to the next free slot
    bool isEmpty() { return head == tail; }
    bool isFull() { return (tail + 1) % MCP::MAX_EVENT == head; }
    size_t advance(size_t index) const { return (index + 1) % MCP::MAX_EVENT; }

    currentEvent getNextEvent(size_t currentIndex) {
      size_t next_index = advance(currentIndex);
      return unresolvedEvents[next_index];
    }

    bool insertEvent(const currentEvent &ev) {
      if (isFull()) {
        return false;
      }

      size_t currentIndex = head;
      while (currentIndex != tail) {
        if (unresolvedEvents[currentIndex].getId() == -1) {
          unresolvedEvents[currentIndex] = ev;
          unresolvedEventCounts[static_cast<size_t>(ev.event)]++;
          return true;
        }
        currentIndex = advance(currentIndex); // Move to the next index
      }

      // If no cleared slot is found, insert at the tail
      unresolvedEvents[tail] = ev;
      tail = advance(tail);
      unresolvedEventCounts[static_cast<size_t>(ev.event)]++;
      return true;
    }

    void decrementEventCount(RegisterEvent eventType) {
      unresolvedEventCounts[static_cast<size_t>(eventType)]--;
    }
    void markEventResolved(size_t index) {
      unresolvedEvents[index]
          .AcknowledgeEvent(); // Mark the event as acknowledged
    }

    void removeResolvedEvents() {
      while (head != tail && unresolvedEvents[head].isAckKnowledged()) {
        unresolvedEventCounts[static_cast<size_t>(
            unresolvedEvents[head].event)]--;
        unresolvedEvents[head] = currentEvent();
        head = advance(head);
      }
    }
    currentEvent getOldestEvent(RegisterEvent eventType) const {
      size_t currentIndex = head;
      while (currentIndex != tail) {
        const auto &event = unresolvedEvents[currentIndex];
        if (event.event == eventType && !event.isAckKnowledged()) {
          return unresolvedEvents[currentIndex];
        }
        currentIndex = advance(currentIndex);
      }
      return currentEvent(); // Return a default event if none found
    }
  };

private:
  static const EventBits_t REGISTER_EVENT_BITS_MASK;
  static SemaphoreHandle_t eventMutex;

  static registerIdentity latest_identity;
  static currentEvent current_event;
  static uint8_t event_counter;
  static eventQueue event_queue;

  static int findOldestEventIndex(RegisterEvent eventType);
  static int findFreeSlot();
  static int getNextId();
  static void cleanupEventGroups();
};

#endif