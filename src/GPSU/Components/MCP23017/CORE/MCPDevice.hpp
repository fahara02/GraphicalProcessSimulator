#ifndef MCP_DEVICE_HPP
#define MCP_DEVICE_HPP

#include "Arduino.h"
#include "MCP_GPIO_banks.hpp"
#include "MCP_Primitives.hpp"
#include "MCP_Registers.hpp"
#include "RegisterEvents.hpp"
#include "Wire.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "i2cBus.hpp"
#include <array>
#include <memory>
#include <tuple>
#include <variant>

#define MCP_TAG "MCPDevice"
namespace MCP {
class InterruptManager;
}
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
  gpio_num_t intA_;
  gpio_num_t intB_;
  MCP::I2CBus &i2cBus_;
  std::unique_ptr<MCP::InterruptManager> interruptManager_;

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
  std::shared_ptr<MCP::Register> cntrlRegA;
  std::shared_ptr<MCP::Register> cntrlRegB;

  MCPDevice(MCP::MCP_MODEL model, bool pinA2 = false, bool pinA1 = false,
            bool pinA0 = false);
  ~MCPDevice();

  void configure(const MCP::Settings &config);
  void setupIntterupt(
      MCP::INTR_TYPE type = MCP::INTR_TYPE::INTR_ON_CHANGE,
      MCP::INTR_OUTPUT_TYPE outtype = MCP::INTR_OUTPUT_TYPE::INTR_ACTIVE_LOW,
      MCP::PairedInterrupt sharedIntr = MCP::PairedInterrupt::Disabled);
  bool enableInterrupt();

  void pinMode(const MCP::Pin pin, const uint8_t mode);
  void pinMode(const int pin, const uint8_t mode);
  void pinMode(const MCP::PORT port, uint8_t pinmask, const uint8_t mode);
  void pinMode(const MCP::PORT port, const uint8_t mode);
  template <
      typename FirstPin, typename... RestPins,
      typename = std::enable_if_t<(std::is_same_v<FirstPin, MCP::Pin> && ... &&
                                   std::is_same_v<RestPins, MCP::Pin>)>>
  void pinMode(const uint8_t mode, FirstPin first, RestPins... rest) {

    uint8_t pinmask = generateMask(first, rest...);
    MCP::PORT port = first.getPort();

    pinMode(port, pinmask, mode);
  }

  void digitalWrite(const int pin, const uint8_t level);
  void digitalWrite(const MCP::Pin pin, const uint8_t level);
  void digitalWrite(const MCP::PORT port, const uint8_t pinmask,
                    const uint8_t level);
  void digitalWrite(const MCP::PORT port, const uint8_t level);
  template <
      typename FirstPin, typename... RestPins,
      typename = std::enable_if_t<(std::is_same_v<FirstPin, MCP::Pin> && ... &&
                                   std::is_same_v<RestPins, MCP::Pin>)>>
  void digitalWrite(const uint8_t level, FirstPin first, RestPins... rest) {
    uint8_t pinmask = generateMask(first, rest...);
    MCP::PORT port = first.getPort();
    return digitalWrite(port, pinmask, static_cast<bool>(level));
  }

  bool digitalRead(const int pin);
  bool digitalRead(const MCP::Pin pin);
  uint8_t digitalRead(const MCP::PORT port, const uint8_t pinmask);
  uint8_t digitalRead(const MCP::PORT port);
  template <
      typename FirstPin, typename... RestPins,
      typename = std::enable_if_t<(std::is_same_v<FirstPin, MCP::Pin> && ... &&
                                   std::is_same_v<RestPins, MCP::Pin>)>>
  uint8_t digitalRead(FirstPin first, RestPins... rest) {
    uint8_t pinmask = generateMask(first, rest...);
    MCP::PORT port = first.getPort();
    return digitalRead(port, pinmask);
  }

  void invertInput(const int pin, bool invert);
  void invertInput(const MCP::Pin pin, bool invert);
  void invertInput(const MCP::PORT port, const uint8_t pinmask, bool invert);
  void invertInput(const MCP::PORT port, bool invert);
  template <
      typename FirstPin, typename... RestPins,
      typename = std::enable_if_t<(std::is_same_v<FirstPin, MCP::Pin> && ... &&
                                   std::is_same_v<RestPins, MCP::Pin>)>>
  void invertInput(FirstPin first, RestPins... rest, bool invert) {

    uint8_t pinmask = generateMask(first, rest...);
    MCP::PORT port = first.getPort();

    return invertInput(port, pinmask, invert);
  }

  int readRegister(MCP::PORT port, MCP::REG regType) const;

  void setupCommunication();

  void dumpRegisters() const;
  MCP::I2CBus &getBus() { return i2cBus_; }

private:
  void init();
  void loadSettings();
  void resetDevice();
  MCP::Register *getRegister(MCP::REG reg, MCP::PORT port);
  uint8_t getsavedSettings(MCP::PORT port) const;

  uint8_t getRegisterAddress(MCP::REG reg, MCP::PORT port) const;

  uint8_t getRegisterSavedValue(MCP::REG reg, MCP::PORT port) const;
  void startEventMonitorTask(MCPDevice *device);
  static void EventMonitorTask(void *param);
  void handleReadEvent(currentEvent *ev);
  void handleWriteEvent(currentEvent *ev);
  void handleSettingChangeEvent(currentEvent *ev);
  void handleBankModeEvent(currentEvent *ev);

  template <typename FirstPin, typename... RestPins>
  constexpr uint8_t generateMask(FirstPin first, RestPins... rest) {
    static_assert(sizeof...(rest) < 8, "Too many pins, max is 8");
    MCP::PORT port = first.getPort();
    assert(((rest.getPort() == port) && ...));

    return (1 << first.getIndex()) | (0 | ... | (1 << rest.getIndex()));
  }
};

} // namespace COMPONENT

#endif
