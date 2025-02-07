#ifndef INTERRUPT_MANAGER_HPP
#define INTERRUPT_MANAGER_HPP
#include "Arduino.h"
#include "MCP_Constants.hpp"
#include "MCP_Registers.hpp"
#include "RegisterEvents.hpp"
#include "Utility.hpp"
#include "i2cBus.hpp"
#include <functional>

#define INT_TAG "INTR_MANAGER"

namespace MCP {
struct InterruptSetting {
  INTR_TYPE intrType = INTR_TYPE::NONE;
  bool intrSharing = false;
  INTR_OUTPUT_TYPE intrOutputType = INTR_OUTPUT_TYPE::NA;
  INTR_ON_CHANGE_CONTROL icoControl = INTR_ON_CHANGE_CONTROL::NA;
  DEF_VAL_COMPARE savedValue = DEF_VAL_COMPARE::NA;
  gpio_int_type_t modeA_ = gpio_int_type_t::GPIO_INTR_DISABLE;
  gpio_int_type_t modeB_ = gpio_int_type_t::GPIO_INTR_DISABLE;
  bool isEnabled = false;
};

class InterruptManager {
private:
  // for user custom callbacks with default interrupt handling
  struct Callback {
    std::function<void()> fn;
    void *userData;
  };
  std::array<Callback, MCP::PIN_PER_BANK> portACallbacks_;
  std::array<Callback, MCP::PIN_PER_BANK> portBCallbacks_;
  struct ISRHandlerData {
    std::function<void(void *)> handler;
    void *userData;
  };
  static ISRHandlerData isrHandlerDataA;
  static ISRHandlerData isrHandlerDataB;

public:
  explicit InterruptManager(MCP::MCP_MODEL m, I2CBus &bus,
                            std::shared_ptr<MCP::Register> iconA,
                            std::shared_ptr<MCP::Register> iconB);
  void setup(INTR_TYPE type = INTR_TYPE::INTR_ON_CHANGE,
             INTR_OUTPUT_TYPE outtype = INTR_OUTPUT_TYPE::INTR_ACTIVE_LOW,
             PairedInterrupt sharedIntr = PairedInterrupt::Disabled);
  void setup(InterruptSetting &setting);
  bool isEnable();
  bool isSharing();
  bool enableInterrupt();
  bool disabeInterrupt();

  Register *getRegister(PORT port, REG reg);
  bool updateRegisterValue(PORT port, uint8_t reg_address, uint8_t value);
  bool resetInterruptRegisters();

  void setupInterruptMask(PORT port, uint8_t mask = 0x00);
  bool updateBankMode(bool value);

  uint8_t getMask(PORT p) const;
  void updateMask(MCP::PORT port, uint8_t mask);

  void invokeCallback(PORT port, uint8_t pinindex);

  InterruptSetting getSetting() const { return setting_; }

  void processPort(MCP::PORT port, uint8_t flag);

  template <typename T>
  void registerCallback(MCP::PORT port, uint8_t pin, void (*func)(T *),
                        T *data) {
    if (pin >= MCP::PIN_PER_BANK)
      return;

    Callback cb;
    cb.fn = [func, data]() { func(data); };
    cb.userData = static_cast<void *>(data);

    if (port == MCP::PORT::GPIOA) {
      portACallbacks_[pin] = cb;
    } else {
      portBCallbacks_[pin] = cb;
    }
  }
  void registerCallback(MCP::PORT port, uint8_t pin, void (*func)(void *)) {
    registerCallback<void>(port, pin, func, nullptr);
  }

  template <typename T>
  void attachMainHandler(PORT port, gpio_num_t espPin,
                         std::function<void(T *)> intHandler, T *userData) {

    if (port == PORT::GPIOA) {
      pinA_ = static_cast<int>(espPin);
      isrHandlerDataA.handler = intHandler;
      isrHandlerDataA.userData = static_cast<void *>(userData);

    } else {
      pinB_ = static_cast<int>(espPin);
      isrHandlerDataB.handler = intHandler;
      isrHandlerDataB.userData = static_cast<void *>(userData);
    }
  }

  void clearInterrupt();
  bool getInterruptFlags(uint8_t &flagA, uint8_t &flagB);
  int getRetryA() { return retryCountA; }
  int getRetryB() { return retryCountB; }
  void retryFlagReadA() {
    retryFalgReadA = true;
    retryCountA = 0;
  }
  void resetRetryA() {
    retryFalgReadA = false;
    retryCountA++;
  }
  bool hasflagReadAFailed() { return retryFalgReadA; }
  void retryFlagReadB() {
    retryFalgReadB = true;
    retryCountB++;
  }
  void resetRetryB() {
    retryFalgReadB = false;
    retryCountB = 0;
  }
  bool hasflagReadBFailed() { return retryFalgReadB; }

private:
  MCP::MCP_MODEL model;
  bool bankMode = false;
  I2CBus &i2cBus_;
  InterruptRegisters regA;
  InterruptRegisters regB;

  InterruptSetting setting_;
  uint8_t maskA_ = 0x00;
  uint8_t maskB_ = 0x00;
  int pinA_ = -1;
  int pinB_ = -1;
  bool retryFalgReadA = false;
  bool retryFalgReadB = false;
  static int retryCountA;
  static int retryCountB;

  bool initIntrGPIOPins();

  void startInterruptTask(InterruptManager *manager);
  static void InterruptProcessorTask(void *param);
  TaskHandle_t intrTaskHandle;
  static SemaphoreHandle_t portATrigger;
  static SemaphoreHandle_t portBTrigger;

  bool updateInterrputSetting();
  bool setupIntteruptOnChnage();
  bool setupEnableRegister();
  bool setupIntrOutput();
  bool setupIntteruptWithDefval(bool savedValue);
  bool confirmRegisterIsSet(PORT port, REG regTpe, uint8_t bitMask);

  static void IRAM_ATTR defaultIntAHandler(void *arg);
  static void IRAM_ATTR defaultIntBHandler(void *arg);
  static void IRAM_ATTR isrWrapper(void *arg) {
    auto data = static_cast<ISRHandlerData *>(arg);
    if (data && data->handler) {
      data->handler(data->userData);
    }
  }

  using Field = Config::Field;
};

} // namespace MCP
#endif