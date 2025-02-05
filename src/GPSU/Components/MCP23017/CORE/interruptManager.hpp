#ifndef INTERRUPT_MANAGER_HPP
#define INTERRUPT_MANAGER_HPP
#include "MCP_Constants.hpp"
#include "MCP_Registers.hpp"
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
  void setup(bool enable, INTR_TYPE type = INTR_TYPE::INTR_ON_CHANGE,
             INTR_OUTPUT_TYPE outtype = INTR_OUTPUT_TYPE::INTR_ACTIVE_LOW,
             PairedInterrupt sharedIntr = PairedInterrupt::Disabled,
             uint8_t mask_A = 0XFF, uint8_t mask_B = 0XFF, int pinA = -1,
             int pinB = -1);
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
  void setupIntteruptOnChnage();
  void setupIntteruptWithDefval(bool savedValue);
};

} // namespace MCP
#endif