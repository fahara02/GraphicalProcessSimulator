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
#include "interruptManager.hpp"
#include <array>
#include <memory>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#define MCP_TAG "MCPDevice"
namespace std {
template <> struct hash<std::tuple<MCP::PORT, MCP::REG>> {
  std::size_t
  operator()(const std::tuple<MCP::PORT, MCP::REG> &key) const noexcept {
    return std::hash<int>()(static_cast<int>(std::get<0>(key))) ^
           (std::hash<int>()(static_cast<int>(std::get<1>(key))) << 1);
  }
};
} // namespace std

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
  MCP::InterruptSetting intrSetting_;

  bool bankMode_ = false;
  bool mirrorMode_ = false;
  bool byteMode_ = false;
  bool slewrateDisabled_ = false;
  bool hardwareAddressing_ = false;
  bool opendrainEnabled_ = false;
  bool interruptPolarityHigh_ = false;

  static SemaphoreHandle_t regRWmutex;
  TaskHandle_t eventTaskHandle;

  std::shared_ptr<MCP::Register> cntrlRegA;
  std::shared_ptr<MCP::Register> cntrlRegB;
  std::unique_ptr<MCP::GPIO_BANK> gpioBankA;
  std::unique_ptr<MCP::GPIO_BANK> gpioBankB;
  std::unique_ptr<MCP::InterruptManager> interruptManager_;
  std::unordered_map<std::tuple<MCP::PORT, MCP::REG>, uint8_t> addressMap_;
  std::function<void(void *)> customIntAHandler_;
  std::function<void(void *)> customIntBHandler_;
  void *userDataA_ = nullptr;
  void *userDataB_ = nullptr;

public:
  MCPDevice(MCP::MCP_MODEL model, bool pinA2 = false, bool pinA1 = false,
            bool pinA0 = false);
  ~MCPDevice();
  void init();
  void configure(const MCP::Settings &config);

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
  void invertInput(bool invert, FirstPin first, RestPins... rest) {

    uint8_t pinmask = generateMask(first, rest...);
    MCP::PORT port = first.getPort();

    return invertInput(port, pinmask, invert);
  }

  void setIntteruptPin(MCP::PORT port, uint8_t pinmask,
                       uint8_t mcpIntrmode = CHANGE,
                       MCP::INTR_OUTPUT_TYPE intrOutMode =
                           MCP::INTR_OUTPUT_TYPE::INTR_ACTIVE_HIGH);

  void setIntteruptPin(MCP::Pin pin, uint8_t mcpIntrmode = CHANGE,
                       MCP::INTR_OUTPUT_TYPE intrOutMode =
                           MCP::INTR_OUTPUT_TYPE::INTR_ACTIVE_HIGH);

  template <
      typename FirstPin, typename... RestPins,
      typename = std::enable_if_t<(std::is_same_v<FirstPin, MCP::Pin> && ... &&
                                   std::is_same_v<RestPins, MCP::Pin>)>>
  void setIntteruptPin(uint8_t mcpIntrmode, MCP::INTR_OUTPUT_TYPE intrOutMode,
                       FirstPin first, RestPins... rest) {
    uint8_t pinmask = generateMask(first, rest...);
    MCP::PORT port = first.getPort();
    setIntteruptPin(port, pinmask, mcpIntrmode, intrOutMode);
  }

  template <typename Pin, typename Callback, typename... Rest>
  auto setMcpInterrupts(Pin &&pin, Callback &&callback, Rest &&...rest)
      -> std::enable_if_t<std::is_same_v<std::decay_t<Pin>, MCP::Pin> &&
                          std::is_invocable_v<Callback, void *>> {

    MCP::PORT port = pin.getPort();
    uint8_t localIndex = pin.getIndex();
    uint8_t mask = (1 << localIndex);

    interruptManager_->updateMask(port, mask);
    interruptManager_->registerCallback(port, localIndex, callback);

    setMcpInterrupts(std::forward<Rest>(rest)...);
  }

  template <typename Pin, typename Callback, typename T, typename... Rest>
  auto setMcpInterrupts(Pin &&pin, Callback &&callback, T *userData,
                        Rest &&...rest)
      -> std::enable_if_t<std::is_same_v<std::decay_t<Pin>, MCP::Pin> &&
                          std::is_invocable_v<Callback, T *>> {

    MCP::PORT port = pin.getPort();
    uint8_t localIndex = pin.getIndex();
    uint8_t mask = (1 << localIndex);

    interruptManager_->updateMask(port, mask);
    interruptManager_->registerCallback<T>(port, localIndex, callback,
                                           userData);

    setMcpInterrupts(std::forward<Rest>(rest)...);
  }

  void setMcpInterrupts(uint8_t mcpIntrmode,
                        MCP::INTR_OUTPUT_TYPE intrOutMode) {
    interruptManager_->updateSetting(mcpIntrmode, intrOutMode);
  }

  // Interrupt  handling on esp32 Side
  void attachInterrupt(gpio_num_t pinA,
                       std::function<void(void *)> intAHandler = nullptr,
                       uint8_t espIntrmode = CHANGE);
  void attachInterrupt(gpio_num_t pinA,
                       std::function<void(void *)> intAHandler = nullptr,
                       uint8_t espIntrmodeA = CHANGE,
                       gpio_num_t pinB = gpio_num_t::GPIO_NUM_NC,
                       uint8_t espIntrmodeB = CHANGE,
                       std::function<void(void *)> intBHandler = nullptr);

  template <typename T>
  void attachInterrupt(gpio_num_t pinA, std::function<void(T *)> intAHandler,
                       uint8_t espIntrmodeA, gpio_num_t pinB,
                       uint8_t espIntrmodeB,
                       std::function<void(T *)> intBHandler, T *userDataA,
                       T *userDataB) {

    if (pinA != static_cast<gpio_num_t>(-1) &&
        pinB != static_cast<gpio_num_t>(-1)) {
      intrSetting_.intrSharing = false;
    }
    intrSetting_.modeA_ = coverIntrMode(espIntrmodeA);
    intrSetting_.modeB_ = coverIntrMode(espIntrmodeB);
    interruptManager_->setup(intrSetting_);
    interruptManager_->attachMainHandler<T>(MCP::PORT::GPIOA, pinA, intAHandler,
                                            userDataA);
    interruptManager_->attachMainHandler<T>(MCP::PORT::GPIOB, pinB, intBHandler,
                                            userDataB);
  }

  void dumpRegisters() const;

private:
  void initGPIOPins();
  void initIntrGPIOPins(gpio_int_type_t modeA, gpio_int_type_t modeB);
  void loadSettings();
  void resetDevice();

  MCP::Register *getGPIORegister(MCP::REG reg, MCP::PORT port);
  MCP::Register *getIntRegister(MCP::REG reg, MCP::PORT port);

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

  std::unordered_map<std::tuple<MCP::PORT, MCP::REG>, uint8_t>
  populateAddressMap(bool bankMode);

  void updateAddressMap(bool bankMode);

  bool resetInterruptRegisters();
  static void IRAM_ATTR defaultIntAHandler(void *arg);
  static void IRAM_ATTR defaultIntBHandler(void *arg);

  void setupDefaultIntterupt(
      MCP::INTR_TYPE type = MCP::INTR_TYPE::INTR_ON_CHANGE,
      MCP::INTR_OUTPUT_TYPE outtype = MCP::INTR_OUTPUT_TYPE::INTR_ACTIVE_HIGH,
      MCP::PairedInterrupt sharedIntr = MCP::PairedInterrupt::Disabled);

  uint8_t getInterruptMask(MCP::PORT port) {
    return interruptManager_->getMask(port);
  }
  void updateInterruptSetting(uint8_t mcpIntrmode,
                              MCP::INTR_OUTPUT_TYPE intrOutMode);
  gpio_int_type_t coverIntrMode(uint8_t mode);
};

} // namespace COMPONENT

#endif
