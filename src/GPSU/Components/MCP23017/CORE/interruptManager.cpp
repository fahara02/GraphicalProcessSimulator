#include "interruptManager.hpp"
#include "MCPDevice.hpp"
namespace MCP {
InterruptManager::InterruptManager(COMPONENT::MCPDevice *owner)
    : owner_(owner),
      gpIntEnA(owner_->gpioBankA->getRegisterForUpdate(REG::GPINTEN)),
      gpIntEnB(owner_->gpioBankB->getRegisterForUpdate(REG::GPINTEN)),
      IntConA(owner_->gpioBankA->getRegisterForUpdate(REG::INTCON)),
      IntConB(owner_->gpioBankB->getRegisterForUpdate(REG::INTCON)),
      intFA(owner_->gpioBankA->getRegisterForUpdate(REG::INTF)),
      intFB(owner_->gpioBankB->getRegisterForUpdate(REG::INTF)),
      intCapA(owner_->gpioBankA->getRegisterForUpdate(REG::INTCAP)),
      intCapB(owner_->gpioBankB->getRegisterForUpdate(REG::INTCAP)),
      defValA(owner_->gpioBankA->getRegisterForUpdate(REG::DEFVAL)),
      defValB(owner_->gpioBankB->getRegisterForUpdate(REG::DEFVAL)) {}

void InterruptManager::setup(bool enable, INTR_TYPE type,
                             INTR_OUTPUT_TYPE outtype,
                             PairedInterrupt sharedIntr, uint8_t mask_A,
                             uint8_t mask_B, int pinA, int pinB) {

  setting_.isEnabled = enable;
  setting_.intrType = type;
  setting_.intrOutputType = outtype;
  setting_.intrSharing = sharedIntr == PairedInterrupt::Enabled ? true : false;
  maskA_ = mask_A;
  maskB_ = mask_B;
  pinA_ = pinA != -1 ? pinA : -1;
  pinB_ = pinB != -1 ? pinB : -1;

  if (setting_.isEnabled) {
    if (setting_.intrSharing) {
      owner_->cntrlRegA->setInterruptSahring<REG::IOCON>(true);
      vTaskDelay(10);
    }
    switch (setting_.intrType) {

    case INTR_TYPE::INTR_ON_CHANGE:
      setupIntteruptOnChnage();
      break;
    case INTR_TYPE::INTR_ON_RISING:
      setupIntteruptWithDefval(false);
      break;
    case INTR_TYPE::INTR_ON_FALLING:
      setupIntteruptWithDefval(true);
      break;
    case INTR_TYPE::NONE:
      break;
    default:
      break;
    };
  }
}

void InterruptManager::setupIntteruptOnChnage() {

  setting_.icoControl = INTR_ON_CHANGE_CONTROL::COMPARE_WITH_OLD_VALUE;
  // No change in INTCONA or B as it is already default 0
  // Enable Intterupt
  gpIntEnA->applyMask(maskA_);
  if (!setting_.intrSharing) {
    gpIntEnB->applyMask(maskB_);
    vTaskDelay(10);
  }
  vTaskDelay(10);

  // Select Intterupt Output Type
  if (setting_.intrOutputType == INTR_OUTPUT_TYPE::INTR_ACTIVE_HIGH) {
    owner_->cntrlRegA->setInterruptPolarity<REG::IOCON>(true);
    vTaskDelay(10);

  } else if (setting_.intrOutputType == INTR_OUTPUT_TYPE::INTR_OPEN_DRAIN) {
    owner_->cntrlRegA->setOpenDrain<REG::IOCON>(true);
    // No Need to change INTPOL bit as ODR=1 will ovveride it
    vTaskDelay(10);
  }
}

void InterruptManager::setupIntteruptWithDefval(bool savedValue) {

  setting_.icoControl = INTR_ON_CHANGE_CONTROL::COMPARE_WITH_DEFVAL;
  // ACTIVATE defval comparison first
  IntConA->applyMask(maskA_);
  vTaskDelay(10);
  if (!setting_.intrSharing) {
    IntConB->applyMask(maskB_);
    vTaskDelay(10);
  }
  // Save the value to be compared with in DEFVAL
  defValA->saveCompareValue<REG::DEFVAL>(
      maskA_, savedValue ? DEF_VAL_COMPARE::SAVE_LOGIC_HIGH
                         : DEF_VAL_COMPARE::SAVE_LOGIC_LOW);
  vTaskDelay(10);
  if (!setting_.intrSharing) {
    defValB->saveCompareValue<REG::DEFVAL>(
        maskB_, savedValue ? DEF_VAL_COMPARE::SAVE_LOGIC_HIGH
                           : DEF_VAL_COMPARE::SAVE_LOGIC_LOW);
    vTaskDelay(10);
  }
  // Enable Intterupt
  gpIntEnA->applyMask(maskA_);
  if (!setting_.intrSharing) {
    gpIntEnB->applyMask(maskB_);
    vTaskDelay(10);
  }
  vTaskDelay(10);
  // Select Intterupt Output Type
  if (setting_.intrOutputType == INTR_OUTPUT_TYPE::INTR_ACTIVE_HIGH) {
    owner_->cntrlRegA->setInterruptPolarity<REG::IOCON>(true);
    vTaskDelay(10);
  } else if (setting_.intrOutputType == INTR_OUTPUT_TYPE::INTR_OPEN_DRAIN) {
    owner_->cntrlRegA->setOpenDrain<REG::IOCON>(true);
    // No Need to change INTPOL as it is default 0
    vTaskDelay(10);
  }
}

} // namespace MCP
