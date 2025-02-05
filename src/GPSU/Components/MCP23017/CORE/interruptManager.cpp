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

void InterruptManager::setup(INTR_TYPE type, INTR_OUTPUT_TYPE outtype,
                             PairedInterrupt sharedIntr, uint8_t mask_A,
                             uint8_t mask_B, int pinA, int pinB) {

  setting_.intrType = type;
  setting_.intrOutputType = outtype;
  setting_.intrSharing = sharedIntr == PairedInterrupt::Enabled ? true : false;
  maskA_ = mask_A;
  maskB_ = mask_B;
  pinA_ = pinA != -1 ? pinA : -1;
  pinB_ = pinB != -1 ? pinB : -1;
}
bool InterruptManager::enableInterrupt() {
  setting_.isEnabled = true;
  if (updateInterrputSetting()) {
    ESP_LOGI(REG_TAG, "SuccessFully intterrupt is set");
    return true;
  } else {
    ESP_LOGE(REG_TAG, "Error in intterupt setup");
    return false;
  }
}
bool InterruptManager::updateInterrputSetting() {
  bool success = false;
  if (setting_.isEnabled) {
    if (setting_.intrSharing) {
      // setting mirror in just one ICON is enough
      owner_->cntrlRegA->setInterruptSahring<REG::IOCON>(true);
      vTaskDelay(10);
      success = confirmRegisterIsSet(PORT::GPIOA, REG::IOCON,
                                     static_cast<uint8_t>(Field::MIRROR));
      vTaskDelay(10);
    }
    switch (setting_.intrType) {

    case INTR_TYPE::INTR_ON_CHANGE:
      success = setupIntteruptOnChnage();

      return success;

    case INTR_TYPE::INTR_ON_RISING:
      success = setupIntteruptWithDefval(false);

      return success;

    case INTR_TYPE::INTR_ON_FALLING:
      success = setupIntteruptWithDefval(true);

      return success;

    case INTR_TYPE::NONE:
    default:
      return success;
    };
  }

  return success;
}

bool InterruptManager::setupIntteruptOnChnage() {
  bool success = false;
  setting_.icoControl = INTR_ON_CHANGE_CONTROL::COMPARE_WITH_OLD_VALUE;
  // No change in INTCONA or B as it is already default 0
  // Enable Intterupt
  success = setupEnableRegister();
  success = setupIntrOutput();
  return success;
}

bool InterruptManager::setupIntteruptWithDefval(bool savedValue) {
  bool success = false;
  setting_.icoControl = INTR_ON_CHANGE_CONTROL::COMPARE_WITH_DEFVAL;
  // ACTIVATE defval comparison first
  IntConA->applyMask(maskA_);
  vTaskDelay(10);
  success = confirmRegisterIsSet(PORT::GPIOA, REG::INTCON, maskA_);
  vTaskDelay(10);
  if (!setting_.intrSharing) {
    IntConB->applyMask(maskB_);
    vTaskDelay(10);
    success = confirmRegisterIsSet(PORT::GPIOB, REG::INTCON, maskB_);
    vTaskDelay(10);
  }
  // Save the value to be compared with in DEFVAL
  defValA->saveCompareValue<REG::DEFVAL>(
      maskA_, savedValue ? DEF_VAL_COMPARE::SAVE_LOGIC_HIGH
                         : DEF_VAL_COMPARE::SAVE_LOGIC_LOW);

  vTaskDelay(10);
  if (savedValue) {
    success = confirmRegisterIsSet(PORT::GPIOA, REG::DEFVAL, maskA_);
    vTaskDelay(10);
  }
  if (!setting_.intrSharing) {
    defValB->saveCompareValue<REG::DEFVAL>(
        maskB_, savedValue ? DEF_VAL_COMPARE::SAVE_LOGIC_HIGH
                           : DEF_VAL_COMPARE::SAVE_LOGIC_LOW);
    vTaskDelay(10);
    if (savedValue) {
      success = confirmRegisterIsSet(PORT::GPIOB, REG::DEFVAL, maskB_);
      vTaskDelay(10);
    }
  }
  // Enable Intterupt
  success = setupEnableRegister();

  // Select Intterupt Output Type
  success = setupIntrOutput();
  return success;
}
bool InterruptManager::setupEnableRegister() {
  bool success = false;
  gpIntEnA->applyMask(maskA_);
  vTaskDelay(10);
  success = confirmRegisterIsSet(PORT::GPIOA, REG::GPINTEN, maskA_);
  vTaskDelay(10);
  if (!setting_.intrSharing) {
    gpIntEnB->applyMask(maskB_);
    vTaskDelay(10);
    success = confirmRegisterIsSet(PORT::GPIOB, REG::GPINTEN, maskB_);
    vTaskDelay(10);
  }
  return success;
}
bool InterruptManager::setupIntrOutput() {
  bool success = false;
  if (setting_.intrOutputType == INTR_OUTPUT_TYPE::INTR_ACTIVE_HIGH) {
    owner_->cntrlRegA->setInterruptPolarity<REG::IOCON>(true);
    vTaskDelay(10);
    success = confirmRegisterIsSet(PORT::GPIOA, REG::IOCON,
                                   static_cast<uint8_t>(Field::INTPOL));
    vTaskDelay(10);
  } else if (setting_.intrOutputType == INTR_OUTPUT_TYPE::INTR_OPEN_DRAIN) {
    owner_->cntrlRegA->setOpenDrain<REG::IOCON>(true);
    // No Need to change INTPOL as it is default 0
    vTaskDelay(10);
    success = confirmRegisterIsSet(PORT::GPIOA, REG::IOCON,
                                   static_cast<uint8_t>(Field::ODR));
    vTaskDelay(10);
    // Setting in ICONA will have same reflection on ICONB so redundant
  } else {
    // No need to write and read IOCON intpol as it is default 0
    success = true;
  }
  return success;
}

bool InterruptManager::confirmRegisterIsSet(PORT port, REG regType,
                                            uint8_t bitMask) {
  bool success = false;
  int currentValue = owner_->readRegister(port, regType);
  if (currentValue == -1) {
    ESP_LOGE(INT_TAG, "Error setting reg %s", Util::ToString::REG(regType));
  } else {

    if (Util::BIT::isSingleBit(bitMask)) {
      bool checkedBit = Util::BIT::isSet(currentValue, bitMask);
      success = checkedBit;
    } else {
      bool chekedMask = Util::BIT::isAllSet(currentValue, bitMask);
      success = chekedMask;
    }
  }
  return success;
}

} // namespace MCP
