#include "interruptManager.hpp"

namespace MCP {
InterruptManager::InterruptManager(MCP::MCP_MODEL m, I2CBus &bus,
                                   std::shared_ptr<MCP::Register> iconA,
                                   std::shared_ptr<MCP::Register> iconB)
    : model(m), i2cBus_(bus), regA(InterruptRegisters(iconA)),
      regB(InterruptRegisters(iconB)) {

  regA.setup(model, PORT::GPIOA, bankMode);
  regB.setup(model, PORT::GPIOA, bankMode);
}

void InterruptManager::setup(INTR_TYPE type, INTR_OUTPUT_TYPE outtype,
                             PairedInterrupt sharedIntr) {

  setting_.intrType = type;
  setting_.intrOutputType = outtype;
  setting_.intrSharing = sharedIntr == PairedInterrupt::Enabled ? true : false;
  ESP_LOGI(INT_TAG, "setting loaded");
}
void InterruptManager::setup(InterruptSetting &setting) {

  setting_.intrType = setting.intrType;
  setting_.intrOutputType = setting.intrOutputType;
  setting_.intrSharing = setting.intrSharing;

  ESP_LOGI(INT_TAG, "setting loaded");
}
void InterruptManager::setupIntteruptMask(PORT port, uint8_t mask) {
  if (port == PORT::GPIOA) {
    maskA_ = mask;
  } else {
    maskB_ = mask;
  }
}

Register *InterruptManager::getRegister(PORT port, REG reg) {
  return port == PORT::GPIOA ? regA.getRegisterForUpdate(reg)
                             : regB.getRegisterForUpdate(reg);
}
bool InterruptManager::updateRegisterValue(PORT port, uint8_t reg_address,
                                           uint8_t value) {
  return port == PORT::GPIOA ? regA.updateRegisterValue(reg_address, value)
                             : regB.updateRegisterValue(reg_address, value);
}

bool InterruptManager::enableInterrupt() {
  setting_.isEnabled = true;
  ESP_LOGI(INT_TAG, "enabling intterupt");
  if (updateInterrputSetting()) {
    ESP_LOGI(INT_TAG, "SuccessFully intterrupt is set");
    return true;
  } else {
    ESP_LOGE(INT_TAG, "Error in intterupt setup");
    return false;
  }
}
bool InterruptManager::updateInterrputSetting() {
  bool success = false;
  if (setting_.isEnabled) {
    if (setting_.intrSharing) {
      // setting mirror in just one ICON is enough
      regA.iocon->setInterruptSahring<REG::IOCON>(true);
      vTaskDelay(10);
      success = confirmRegisterIsSet(PORT::GPIOA, REG::IOCON,
                                     static_cast<uint8_t>(Field::MIRROR));
      vTaskDelay(10);
    }
    switch (setting_.intrType) {
      ESP_LOGI(INT_TAG, "checking intterupt types");
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
bool InterruptManager::resetInterruptRegisters() {
  ESP_LOGI("MCP_DEVICE", "resetting all interrupts ");

  bool status = true;

  status &=
      (i2cBus_.write_mcp_register(regA.gpinten->getAddress(), 0x00, true) == 0);

  status &=
      (i2cBus_.write_mcp_register(regB.gpinten->getAddress(), 0x00, true) == 0);

  status &=
      (i2cBus_.write_mcp_register(regA.intcon->getAddress(), 0x00, true) == 0);
  status &=
      (i2cBus_.write_mcp_register(regB.intcon->getAddress(), 0x00, true) == 0);

  status &= (i2cBus_.read_mcp_register(regA.intcap->getAddress(), true) == 0);
  status &= (i2cBus_.read_mcp_register(regB.intcap->getAddress(), true) == 0);

  return status;
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
  regA.intcon->applyMask(maskA_);
  vTaskDelay(10);
  success = confirmRegisterIsSet(PORT::GPIOA, REG::INTCON, maskA_);
  vTaskDelay(10);
  if (!setting_.intrSharing) {
    regB.intcon->applyMask(maskB_);
    vTaskDelay(10);
    success = confirmRegisterIsSet(PORT::GPIOB, REG::INTCON, maskB_);
    vTaskDelay(10);
  }
  // Save the value to be compared with in DEFVAL
  regA.defval->saveCompareValue<REG::DEFVAL>(
      maskA_, savedValue ? DEF_VAL_COMPARE::SAVE_LOGIC_HIGH
                         : DEF_VAL_COMPARE::SAVE_LOGIC_LOW);

  vTaskDelay(10);
  if (savedValue) {
    success = confirmRegisterIsSet(PORT::GPIOA, REG::DEFVAL, maskA_);
    vTaskDelay(10);
  }
  if (!setting_.intrSharing) {
    regB.defval->saveCompareValue<REG::DEFVAL>(
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

  regA.gpinten->applyMask(maskA_);
  vTaskDelay(10);
  success = confirmRegisterIsSet(PORT::GPIOA, REG::GPINTEN, maskA_);
  vTaskDelay(10);
  if (!setting_.intrSharing) {
    regB.gpinten->applyMask(maskB_);
    vTaskDelay(10);
    success = confirmRegisterIsSet(PORT::GPIOB, REG::GPINTEN, maskB_);
    vTaskDelay(10);
  }
  return success;
}
bool InterruptManager::setupIntrOutput() {
  bool success = false;

  if (setting_.intrOutputType == INTR_OUTPUT_TYPE::INTR_ACTIVE_HIGH) {
    regA.iocon->setInterruptPolarity<REG::IOCON>(true);
    vTaskDelay(10);
    success = confirmRegisterIsSet(PORT::GPIOA, REG::IOCON,
                                   static_cast<uint8_t>(Field::INTPOL));
    vTaskDelay(10);
  } else if (setting_.intrOutputType == INTR_OUTPUT_TYPE::INTR_OPEN_DRAIN) {
    regA.iocon->setOpenDrain<REG::IOCON>(true);
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

  uint8_t address =
      port == PORT::GPIOA ? regA.getAddress(regType) : regB.getAddress(regType);

  int currentValue = i2cBus_.read_mcp_register(address, bankMode);
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

bool InterruptManager::updateBankMode(bool value) {
  // icon register not need to be invoked again as this method is already
  // invoked and icon is updated  ,just notify this class for updating address
  bankMode = value;
  regA.updateAddress(bankMode);
  regB.updateAddress(bankMode);
  return true;
}

void InterruptManager::attachInterrupt(int pin,
                                       std::function<void(void *)> callback) {
  if (setting_.intrSharing) {
    // Shared interrupt configuration: use only pinA (if valid)
    if (pinA_ != -1) {
      callbackA_ = callback;
      ESP_LOGI(INT_TAG, "Shared interrupt: callback attached to pinA (%d)",
               pinA_);
    } else {
      ESP_LOGW(INT_TAG, "Shared interrupt enabled but pinA is invalid (-1).");
    }
  } else {
    // Non-shared interrupt configuration: assign based on the given pin
    if (pin == pinA_ && pinA_ != -1) {
      callbackA_ = callback;
      ESP_LOGI(INT_TAG, "Callback attached to pinA (%d)", pinA_);
    } else if (pin == pinB_ && pinB_ != -1) {
      callbackB_ = callback;
      ESP_LOGI(INT_TAG, "Callback attached to pinB (%d)", pinB_);
    } else {
      ESP_LOGW(INT_TAG,
               "attachInterrupt: Unknown or invalid pin %d (expected %d for "
               "pinA or %d for pinB).",
               pin, pinA_, pinB_);
    }
  }
}

void IRAM_ATTR InterruptManager::globalInterruptHandler(void *arg) {

  InterruptManager *intrMgr = static_cast<InterruptManager *>(arg);
  int triggeredPin = /* getTriggeredPin() */ 0;
  if (triggeredPin == intrMgr->pinA_ && intrMgr->callbackA_) {
    intrMgr->callbackA_(nullptr); // Pass any required argument.
  } else if (triggeredPin == intrMgr->pinB_ && intrMgr->callbackB_) {
    intrMgr->callbackB_(nullptr);
  } else {
  }
}

} // namespace MCP
