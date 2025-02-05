#ifndef INTERRUPT_MANAGER_HPP
#define INTERRUPT_MANAGER_HPP
#include "MCP_Constants.hpp"
#include "MCP_Registers.hpp"
#include "RegisterEvents.hpp"
#include "Utility.hpp"
#include "i2cBus.hpp"

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
  void setup(int pinA = -1, int pinB = -1,
             INTR_TYPE type = INTR_TYPE::INTR_ON_CHANGE,
             INTR_OUTPUT_TYPE outtype = INTR_OUTPUT_TYPE::INTR_ACTIVE_LOW,
             PairedInterrupt sharedIntr = PairedInterrupt::Disabled);
  bool enableInterrupt();
  void setupIntteruptMask(uint8_t maskA, uint8_t maskB = 0X00);

  bool updateBankMode(bool value);

  uint8_t getIntrFlagA();
  uint8_t getIntrFlagB();

private:
  MCP::MCP_MODEL model;
  bool bankMode = false;
  I2CBus &i2cBus_;
  InterruptRegisters regA;
  InterruptRegisters regB;

  InterruptSetting setting_;
  uint8_t maskA_ = 0x00;
  uint8_t maskB_ = 0x00;
  int pinA_ = 0;
  int pinB_ = 0;

  bool updateInterrputSetting();
  bool setupIntteruptOnChnage();
  bool setupEnableRegister();
  bool setupIntrOutput();
  bool setupIntteruptWithDefval(bool savedValue);
  bool confirmRegisterIsSet(PORT port, REG regTpe, uint8_t bitMask);
  bool checkRegistersExists();
  using Field = Config::Field;
};

} // namespace MCP
#endif