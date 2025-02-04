#ifndef I2C_BUS_HPP
#define I2C_BUS_HPP
#include "Arduino.h"
#include "MCP_Constants.hpp"
#include "RegisterEvents.hpp"
#include "Wire.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <memory>

namespace MCP {

class I2CBus {

public:
  static I2CBus &getInstance(uint8_t addr, int sda = 25, int scl = 33) {
    static I2CBus instance{addr, sda, scl};

    return instance;
  }
  void setPin(int sda, int scl);
  void init();
  I2CBus(const I2CBus &) = delete;
  I2CBus &operator=(const I2CBus &) = delete;
  I2CBus(I2CBus &&) = delete;
  I2CBus &operator=(I2CBus &&) = delete;

  void startEventMonitorTask(I2CBus *bus);
  static void EventMonitorTask(void *param);
  void handleReadEvent(currentEvent *ev);
  void handleWriteEvent(currentEvent *ev);
  void handleSettingChangeEvent(currentEvent *ev);
  void handleBankModeEvent(currentEvent *ev);
  int read_mcp_register(const uint8_t reg, bool bankMode);
  uint8_t write_mcp_register(const uint8_t reg, uint16_t value, bool bankMode);
  static SemaphoreHandle_t regRWmutex;

private:
  I2CBus(uint8_t addr, int sda, int scl);
  uint8_t address_;
  int sda_;
  int scl_;
  std::unique_ptr<TwoWire> wire_;

  TaskHandle_t eventTaskHandle;
};

} // namespace MCP
#endif