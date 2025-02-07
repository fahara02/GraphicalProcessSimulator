#ifndef INTERRUPT_MANAGER_HPP
#define INTERRUPT_MANAGER_HPP
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

  bool isEnabled = false;
};

class InterruptManager {
private:
  struct Callback {
    std::function<void()> fn;
    void *userData;
  };
  std::array<Callback, MCP::PIN_PER_BANK> portACallbacks_;
  std::array<Callback, MCP::PIN_PER_BANK> portBCallbacks_;

  // std::array<std::function<void(void *)>, MCP::PIN_PER_BANK> portACallbacks_;
  // std::array<std::function<void(void *)>, MCP::PIN_PER_BANK> portBCallbacks_;

public:
  void *userDataA_;
  void *userDataB_;

  explicit InterruptManager(MCP::MCP_MODEL m, I2CBus &bus,
                            std::shared_ptr<MCP::Register> iconA,
                            std::shared_ptr<MCP::Register> iconB);
  void setup(INTR_TYPE type = INTR_TYPE::INTR_ON_CHANGE,
             INTR_OUTPUT_TYPE outtype = INTR_OUTPUT_TYPE::INTR_ACTIVE_LOW,
             PairedInterrupt sharedIntr = PairedInterrupt::Disabled);
  void setup(InterruptSetting &setting);
  bool enableInterrupt();

  uint8_t getIntrFlagA();
  uint8_t getIntrFlagB();
  Register *getRegister(PORT port, REG reg);
  bool updateRegisterValue(PORT port, uint8_t reg_address, uint8_t value);
  bool resetInterruptRegisters();

  void setupInterruptMask(PORT port, uint8_t mask = 0x00);
  bool updateBankMode(bool value);

  uint8_t getMask(PORT p) { return p == PORT::GPIOA ? maskA_ : maskB_; }

  // void setCallback(MCP::PORT port, uint8_t index,
  //                  std::function<void(void *)> callback) {
  //   if (port == MCP::PORT::GPIOA) {
  //     portACallbacks_[index] = std::move(callback);
  //   } else {
  //     portBCallbacks_[index] = std::move(callback);
  //   }
  // }

  void updateMask(MCP::PORT port, uint8_t mask) {
    if (port == MCP::PORT::GPIOA) {
      maskA_ |= mask;
    } else {
      maskB_ |= mask;
    }
  }

  // static void IRAM_ATTR globalInterruptHandler(void *arg);
  void invokeCallback(PORT port, uint8_t pinindex);

  void setUserData(PORT port, void *userData) {

    if (port == PORT::GPIOA) {
      userDataA_ = userData;
    } else {
      userDataB_ = userData;
    }
  }
  void *getUserData(PORT port) {
    return port == PORT::GPIOA ? userDataA_ : userDataB_;
  }
  InterruptSetting getSetting() const { return setting_; }

  void IRAM_ATTR handleInterrupt(MCP::PORT port);
  void IRAM_ATTR processPort(MCP::PORT port, uint8_t flag);

  void registerCallback(MCP::PORT port, uint8_t pin, void (*func)(void *)) {
    registerCallback<void>(port, pin, func, nullptr);
  }

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

  bool updateInterrputSetting();
  bool setupIntteruptOnChnage();
  bool setupEnableRegister();
  bool setupIntrOutput();
  bool setupIntteruptWithDefval(bool savedValue);
  bool confirmRegisterIsSet(PORT port, REG regTpe, uint8_t bitMask);

  using Field = Config::Field;
};

} // namespace MCP
#endif