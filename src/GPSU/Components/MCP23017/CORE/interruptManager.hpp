#ifndef INTERRUPT_MANAGER_HPP
#define INTERRUPT_MANAGER_HPP
#include "MCP_Constants.hpp"
#include "MCP_Registers.hpp"
#include "RegisterEvents.hpp"
#include "Utility.hpp"
#include "i2cBus.hpp"

#define INT_TAG "INTR_MANAGER"
namespace COMPONENT {
class MCPDevice;
}

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
  explicit InterruptManager(COMPONENT::MCPDevice *owner);
  void setup(INTR_TYPE type = INTR_TYPE::INTR_ON_CHANGE,
             INTR_OUTPUT_TYPE outtype = INTR_OUTPUT_TYPE::INTR_ACTIVE_LOW,
             PairedInterrupt sharedIntr = PairedInterrupt::Disabled,
             uint8_t mask_A = 0XFF, uint8_t mask_B = 0XFF, int pinA = -1,
             int pinB = -1);
  bool enableInterrupt();
  uint8_t getIntrFlagA();
  uint8_t getIntrFlagB();

private:
  COMPONENT::MCPDevice *owner_;

  Register *gpIntEnA;
  Register *gpIntEnB;
  Register *IntConA;
  Register *IntConB;

  Register *intFA;
  Register *intFB;
  Register *intCapA;
  Register *intCapB;
  Register *defValA;
  Register *defValB;

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
  using Field = Config::Field;
};

} // namespace MCP
#endif