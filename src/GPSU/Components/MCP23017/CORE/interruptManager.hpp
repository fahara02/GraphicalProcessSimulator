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

public:
  std::function<void(void *)> callbackA_;
  std::function<void(void *)> callbackB_;
  std::array<std::function<void(void *)>, MCP::PIN_PER_BANK> portACallbacks_;
  std::array<std::function<void(void *)>, MCP::PIN_PER_BANK> portBCallbacks_;

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
  void attachInterrupt(int pin, std::function<void(void *)>);

  void setupInterruptMask(PORT port, uint8_t mask = 0x00);
  bool updateBankMode(bool value);

  uint8_t getMask(PORT p) { return p == PORT::GPIOA ? maskA_ : maskB_; }

  void setCallback(MCP::PORT port, uint8_t index,
                   std::function<void(void *)> callback) {
    if (port == MCP::PORT::GPIOA) {
      portACallbacks_[index] = std::move(callback);
    } else {
      portBCallbacks_[index] = std::move(callback);
    }
  }

  template <typename T>
  void registerCallback(MCP::PORT port, uint8_t pin, void (*handler)(T *),
                        T *data);

  void updateMask(MCP::PORT port, uint8_t mask) {
    if (port == MCP::PORT::GPIOA) {
      maskA_ |= mask;
    } else {
      maskB_ |= mask;
    }
  }

  static void IRAM_ATTR globalInterruptHandler(void *arg);
  static void invokeCallback(PORT port, uint8_t pinindex, void *arg);
  void setUserData(PORT port, void *userData) {
    if (port == PORT::GPIOA) {
      userDataA_ = userData;
    } else {
      userDataB_ = userData;
    }
  }
  InterruptSetting getSetting() const { return setting_; }
  bool isINTA() { return pinA_ != (-1); }
  bool isINTB() { return pinB_ != (-1); }
  bool isINTSharing() { return setting_.intrSharing; }
  void *getUserData(PORT port) {
    return port == PORT::GPIOA ? userDataA_ : userDataB_;
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
  void *userDataA_ = nullptr;
  void *userDataB_ = nullptr;

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