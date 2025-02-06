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
  explicit InterruptManager(MCP::MCP_MODEL m, I2CBus &bus,
                            std::shared_ptr<MCP::Register> iconA,
                            std::shared_ptr<MCP::Register> iconB);
  void setup(INTR_TYPE type = INTR_TYPE::INTR_ON_CHANGE,
             INTR_OUTPUT_TYPE outtype = INTR_OUTPUT_TYPE::INTR_ACTIVE_LOW,
             PairedInterrupt sharedIntr = PairedInterrupt::Disabled);
  void setup(InterruptSetting &setting);
  bool enableInterrupt();
  void setupIntteruptMask(PORT port, uint8_t mask = 0x00);

  bool updateBankMode(bool value);

  uint8_t getIntrFlagA();
  uint8_t getIntrFlagB();
  Register *getRegister(PORT port, REG reg);
  bool updateRegisterValue(PORT port, uint8_t reg_address, uint8_t value);
  bool resetInterruptRegisters();
  void attachInterrupt(int pin, std::function<void(void *)>);

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
  std::function<void(void *)> callbackA_;
  std::function<void(void *)> callbackB_;

  bool updateInterrputSetting();
  bool setupIntteruptOnChnage();
  bool setupEnableRegister();
  bool setupIntrOutput();
  bool setupIntteruptWithDefval(bool savedValue);
  bool confirmRegisterIsSet(PORT port, REG regTpe, uint8_t bitMask);

  static void IRAM_ATTR globalInterruptHandler(void *arg);
  using Field = Config::Field;
};

} // namespace MCP
#endif