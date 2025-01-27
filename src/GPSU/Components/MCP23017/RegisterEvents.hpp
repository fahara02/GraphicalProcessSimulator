#ifndef REGISTER_EVENTS_HPP
#define REGISTER_EVENTS_HPP
#include "MCP_Constants.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "stdint.h"

enum class RegisterEvent : EventBits_t {
  NONE = 1 << 0,
  BANK_MODE_CHANGED = 1 << 1,
  READ_REQUEST = 1 << 2,
  WRITE_REQUEST = 1 << 3,
  SETTINGS_CHANGED = 1 << 4,
  CLEAR_INTERRUPT = 1 << 5,
  NETWORK_ERROR = 1 << 6,
  RESTART = 1 << 7
};
struct registerIdentity {
  MCP::REG reg;
  MCP::PORT port;
  uint8_t regAddress;

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
  const uint8_t data;
  currentEvent(RegisterEvent e, registerIdentity identity,
               uint8_t valueOrSettings = 0, uint8_t id)
      : event(e), regIdentity(identity), data(valueOrSettings), id_(id),
        acknowledged_(false){

        };
  void AcknowledgeEvent() { acknowledged = true; }
  bool isAckKnowledged() { return acknowledged; }
  uint8_t getId const() { return id_; }

private:
  const uint8_t id_ = 0;
  bool acknowledged_;
};
class EventManager {
public:
  static EventGroupHandle_t registerEventGroup;
  static SemaphoreHandle_t eventMutex;
  static void setBits(RegisterEvent e);
  static void clearBits(RegisterEvent e);
  static void initializeEventGroups();
  static void createEvent(registerIdentity identity;
                          uint8_t valueOrSettings = 0);
  static currentEvent &getCurrentEvent();

protected:
  static void cleanupEventGroups();

private:
  static const EventBits_t REGISTER_EVENT_BITS_MASK;

  uint8_t getEventId() {
    if (event_counter < MAX_EVENT) {
      event_counter += event_counter;
    } else {
      event_counter = 0;
    }
  }
  static registerIdentity latest_identity;
  static currentEvent current_event;

  std::array<currentEvent, MAX_EVENT> allEvents;
  static uint8_t event_counter = 0;
};

#endif