#ifndef INTERRUPT_MANAGER_HPP
#define INTERRUPT_MANAGER_HPP
#include "MCP_Constants.hpp"
namespace COMPONENT {
class MCPDevice;
}

namespace MCP {
struct InterruptSetting {
  INTR_TYPE intType = INTR_TYPE::NONE;
  PairedInterrupt intSharing = PairedInterrupt::Disabled;
  INTR_OUTPUT_TYPE intOutputType = INTR_OUTPUT_TYPE::NA;
  INTR_ON_CHANGE_CONTROL icoControl = INTR_ON_CHANGE_CONTROL::NA;
  DEF_VAL_COMPARE savedValue = DEF_VAL_COMPARE::NA;
  uint8_t maskA = 0x00;
  uint8_t maskB = 0x00;
  bool isEnabled = false;
};

class InterruptManager {

public:
  explicit InterruptManager(COMPONENT::MCPDevice *owner);

private:
  COMPONENT::MCPDevice *owner_;
  InterruptSetting setting_;
};

} // namespace MCP
#endif