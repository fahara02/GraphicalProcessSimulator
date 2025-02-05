#ifndef REGISTER_EVENTS_HPP
#define REGISTER_EVENTS_HPP

#include "Arduino.h"
#include "MCP_Constants.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include <array>
#include <atomic>
#include <memory>
#include <unordered_map>

enum class RegisterEvent : EventBits_t {
  BANK_MODE_CHANGED = 1 << 0,
  READ_REQUEST = 1 << 1,
  WRITE_REQUEST = 1 << 2,
  SETTINGS_CHANGED = 1 << 3,
  CLEAR_INTERRUPT = 1 << 4,
  DATA_RECEIVED = 1 << 5,
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

  bool operator==(const registerIdentity &other) const {
    return (reg == other.reg) && (port == other.port) &&
           (regAddress == other.regAddress);
  }
};

namespace std {
template <> struct hash<registerIdentity> {
  size_t operator()(const registerIdentity &key) const {
    return static_cast<size_t>(key.reg) ^ static_cast<size_t>(key.port) ^
           static_cast<size_t>(key.regAddress);
  }
};
} // namespace std

struct currentEvent {
  RegisterEvent event;
  registerIdentity regIdentity;
  uint16_t data;
  int id;
  bool intteruptFunction_;
  bool acknowledged_;

  currentEvent(RegisterEvent e = RegisterEvent::MAX,
               registerIdentity identity = registerIdentity{},
               uint16_t valueOrSettings = 0, uint8_t _id = 0,
               bool intrFn = false)
      : event(e), regIdentity(identity), data(valueOrSettings), id(_id),
        intteruptFunction_(intrFn), acknowledged_(false) {}

  bool isIdentical(const RegisterEvent &e, const registerIdentity &identity,
                   uint8_t valueOrSettings) const {
    return (event == e) && (regIdentity == identity) &&
           (data == valueOrSettings);
  }

  void AcknowledgeEvent() { acknowledged_ = true; }
  bool isAcknowledged() const { return acknowledged_; }
};

class EventManager {
public:
  static EventGroupHandle_t registerEventGroup;

  static void setBits(RegisterEvent e);
  static void clearBits(RegisterEvent e);
  static void initializeEventGroups();
  static bool createEvent(registerIdentity identity, RegisterEvent e,
                          uint16_t valueOrSettings = 0, bool intrFn = false);
  static currentEvent *getEvent(RegisterEvent eventType);
  static bool acknowledgeEvent(currentEvent *event);
  static size_t getQueueSize();

private:
  static const EventBits_t REGISTER_EVENT_BITS_MASK;

  static constexpr size_t MAX_EVENTS = MCP::MAX_EVENT;
  static std::array<currentEvent, MAX_EVENTS> eventBuffer;
  static std::atomic<size_t> head;
  static std::atomic<size_t> tail;
  static std::unordered_map<registerIdentity, size_t> eventIndexMap;
};

#endif // REGISTER_EVENTS_HPP
