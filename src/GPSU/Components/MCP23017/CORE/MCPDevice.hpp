#ifndef MCP_DEVICE_HPP
#define MCP_DEVICE_HPP

#include "Arduino.h"
#include "MCP_GPIO_banks.hpp"
#include "MCP_I2Cbus.hpp"
#include "MCP_Primitives.hpp"
#include "MCP_Registers.hpp"
#include "RegisterEvents.hpp"
#include "Wire.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <array>
#include <memory>
#include <variant>

#define MCP_TAG "MCPDevice"

namespace COMPONENT {

class MCPDevice {
private:
  MCP::MCP_MODEL model_;
  MCP::Config configuration_;
  MCP::Settings settings_;
  MCP::Settings defaultSettings_;
  MCP::address_decoder_t decoder_;
  uint8_t address_;
  gpio_num_t sda_;
  gpio_num_t scl_;
  gpio_num_t cs_;
  gpio_num_t reset_;
  std::unique_ptr<TwoWire> wire_;
  bool bankMode_ = false;
  bool mirrorMode_ = false;
  bool byteMode_ = false;
  bool slewrateDisabled_ = false;
  bool hardwareAddressing_ = false;
  bool opendrainEnabled_ = false;
  bool interruptPolarityHigh_ = false;

  static SemaphoreHandle_t regRWmutex;
  TaskHandle_t eventTaskHandle;

public:
  std::unique_ptr<MCP::GPIO_BANK> gpioBankA;
  std::unique_ptr<MCP::GPIO_BANK> gpioBankB;
  std::shared_ptr<MCP::MCPRegister> cntrlRegA;
  std::shared_ptr<MCP::MCPRegister> cntrlRegB;

  MCPDevice(MCP::MCP_MODEL model, bool pinA2 = false, bool pinA1 = false,
            bool pinA0 = false);

  void configure(const MCP::Settings &config);

  void pinMode(MCP::Pin pin, const uint8_t mode);
  void pinMode(const int pin, const uint8_t mode);
  void pinMode(MCP::PORT port, uint8_t pinmask, const uint8_t mode);
  void pinMode(MCP::PORT port, const uint8_t mode);

  void setupCommunication();

  void dumpRegisters() const;

private:
  void init();
  void loadSettings();
  void resetDevice();
  MCP::MCPRegister *getRegister(MCP::REG reg, MCP::PORT port);
  uint8_t getsavedSettings(MCP::PORT port) const;
  void updateRegisters(MCPDevice *device);
  uint8_t getRegisterAddress(MCP::REG reg, MCP::PORT port) const;

  uint8_t getRegisterSavedValue(MCP::REG reg, MCP::PORT port) const;
  void startEventMonitorTask(MCPDevice *device);
  static void EventMonitorTask(void *param);
  void handleReadEvent(currentEvent *ev);
  void handleWriteEvent(currentEvent *ev);
  void handleSettingChangeEvent(currentEvent *ev);
  void handleBankModeEvent(currentEvent *ev);
  int read_mcp_register(const uint8_t reg);
  uint8_t write_mcp_register(const uint8_t reg, uint16_t value);

  void read_mcp_registers_batch(uint8_t startReg, uint8_t *data, size_t length);
  void write_mcp_registers_batch(uint8_t startReg, const uint8_t *data,
                                 size_t length);
};

} // namespace COMPONENT

#endif
