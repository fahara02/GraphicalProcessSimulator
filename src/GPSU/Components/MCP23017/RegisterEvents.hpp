#ifndef REGISTER_EVENTS_HPP
#define REGISTER_EVENTS_HPP
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
struct currentEvent {
  RegisterEvent event;
  uint8_t regAddress;
  uint8_t value = 0;
  uint8_t settings = 0;
};
class EventManager {
public:
  static EventGroupHandle_t registerEventGroup;
  static SemaphoreHandle_t eventMutex;
  static void setBits(RegisterEvent e);
  static void clearBits(RegisterEvent e);
  static void initializeEventGroups();
  static void createEvent(uint8_t addr, RegisterEvent e, uint8_t value = 0,
                          uint8_t settings = 0);

protected:
  static void cleanupEventGroups();
  static currentEvent &getCurrentEvent();

private:
  static const EventBits_t REGISTER_EVENT_BITS_MASK;
  static currentEvent current_event;
};

#endif