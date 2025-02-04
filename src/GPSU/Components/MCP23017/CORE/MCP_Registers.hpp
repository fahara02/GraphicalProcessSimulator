#ifndef MCP_REGISTERS_HPP
#define MCP_REGISTERS_HPP
#include "MCP_Constants.hpp"
#include "MCP_Primitives.hpp"
#include "RegisterEvents.hpp"
#include "Utility.hpp"
#include "esp_log.h"
#include "freertos/semphr.h"
#include "memory"
#include <functional>
#include <type_traits>
#include <unordered_map>

#define REG_TAG "MCP_REGISTERS"
namespace MCP {

struct Settings {
  OperationMode opMode = OperationMode::SequentialMode16;
  PairedInterrupt mirror = PairedInterrupt::Disabled;
  Slew slew = Slew::Enabled;
  HardwareAddr haen = HardwareAddr::Disabled;
  OpenDrain odr = OpenDrain::Disabled;
  InterruptPolarity intpol = InterruptPolarity::ActiveLow;

  explicit Settings(MCP::MCP_MODEL m = MCP::MCP_MODEL::MCP23017) : model_(m) {}

  bool operator==(const Settings &other) const {
    return opMode == other.opMode && mirror == other.mirror &&
           slew == other.slew && haen == other.haen && odr == other.odr &&
           intpol == other.intpol && model_ == other.model_;
  }

  bool operator!=(const Settings &other) const { return !(*this == other); }

  uint8_t getSetting() const {
    return (static_cast<uint8_t>(opMode == OperationMode::SequentialMode8 ||
                                 opMode == OperationMode::ByteMode8)
            << 7) |
           (static_cast<uint8_t>(mirror) << 6) |
           (static_cast<uint8_t>(opMode == OperationMode::ByteMode16 ||
                                 opMode == OperationMode::ByteMode8)
            << 5) |
           (static_cast<uint8_t>(slew) << 4) |
           (static_cast<uint8_t>(haen) << 3) |
           (static_cast<uint8_t>(odr) << 2) |
           (static_cast<uint8_t>(intpol) << 1);
  }

  void setModel(MCP::MCP_MODEL m) { model_ = m; }
  MCP::MCP_MODEL getModel() const { return model_; }

private:
  MCP::MCP_MODEL model_;
};

// ================= Config Struct ==================
struct Config {
  enum Field : uint8_t {
    RESERVED,
    INTPOL,
    ODR,
    HAEN,
    DISSLW,
    SEQOP,
    MIRROR,
    BANK
  };

  explicit Config(MCP::MCP_MODEL m = MCP::MCP_MODEL::MCP23017)
      : config_(m), value(0) {}

  void configureDefault() { validate_update(config_); }

  bool configure(const Settings &setting) {
    Settings validated = setting;
    applyValidationRules(validated);
    return validate_update(validated);
  }

  bool configure(uint8_t new_setting) {
    return configure(extractSettings(new_setting));
  }

  uint8_t getSettingValue() const { return value; }
  Settings getSettings() const { return config_; }

  bool getBitField(Field field) const { return value & (1 << field); }

  void setOperationMode(bool byteMode, bool mapping8Bit) {
    config_.opMode = byteMode ? (mapping8Bit ? OperationMode::ByteMode8
                                             : OperationMode::ByteMode16)
                              : (mapping8Bit ? OperationMode::SequentialMode8
                                             : OperationMode::SequentialMode16);
    updateConfig();
  }

  void setMirror(bool enable) { updateSetting(config_.mirror, enable); }
  void setSlewRate(bool enable) { updateSetting(config_.slew, enable); }
  void setOpenDrain(bool enable) { updateSetting(config_.odr, enable); }
  void setInterruptPolarity(bool activeHigh) {
    updateSetting(config_.intpol, activeHigh);
  }

  void setHardwareAddressing(bool enable) {
    if (config_.getModel() == MCP_MODEL::MCP23017 ||
        config_.getModel() == MCP_MODEL::MCP23018) {
      enable = false;
    }
    updateSetting(config_.haen, enable);
  }

private:
  Settings config_;
  uint8_t value;

  void updateConfig() { value = config_.getSetting(); }

  void applyValidationRules(Settings &setting) {
    if (setting.getModel() == MCP_MODEL::MCP23017 ||
        setting.getModel() == MCP_MODEL::MCP23018) {
      setting.haen = HardwareAddr::Disabled;
    }
    if (setting.odr == OpenDrain::Enabled &&
        setting.intpol == InterruptPolarity::ActiveHigh) {
      setting.intpol = InterruptPolarity::ActiveLow;
    }
  }

  bool validate_update(Settings &setting) {
    if (config_ == setting)
      return false;
    config_ = setting;
    updateConfig();
    return true;
  }

  Settings extractSettings(uint8_t setting) {
    Settings new_config(config_.getModel());
    new_config.opMode =
        (setting & (1 << 7))
            ? ((setting & (1 << 5)) ? OperationMode::ByteMode8
                                    : OperationMode::SequentialMode8)
            : ((setting & (1 << 5)) ? OperationMode::ByteMode16
                                    : OperationMode::SequentialMode16);
    new_config.mirror = static_cast<PairedInterrupt>(setting & (1 << 6));
    new_config.slew = static_cast<Slew>(!(setting & (1 << 4)));
    new_config.haen = static_cast<HardwareAddr>(setting & (1 << 3));
    new_config.odr = static_cast<OpenDrain>(setting & (1 << 2));
    new_config.intpol = static_cast<InterruptPolarity>(setting & (1 << 1));

    return new_config;
  }

  template <typename T> void updateSetting(T &field, bool enable) {
    if (field != static_cast<T>(enable)) {
      field = static_cast<T>(enable);
      updateConfig();
    }
  }
};

struct address_decoder_t {
  const MCP::MCP_MODEL model_;
  const bool A2_;
  const bool A1_;
  const bool A0_;

  address_decoder_t(MCP::MCP_MODEL m, bool A2, bool A1, bool A0)
      : model_(m), A2_(A2), A1_(A1), A0_(A0),
        device_i2c_address(decodeDeviceAddress()) {}

  constexpr uint8_t decodeDeviceAddress() {
    if (model_ == MCP::MCP_MODEL::MCP23017 ||
        model_ == MCP::MCP_MODEL::MCP23S17) {
      return MCP_ADDRESS_BASE | (A2_ << 2) | (A1_ << 1) | A0_;
    }
    return MCP_ADDRESS_BASE;
  }
  uint8_t getDeviceAddress() const { return device_i2c_address; }

private:
  uint8_t device_i2c_address;
};

//
class RegisterBase {

public:
  using Callback = std::function<void(uint8_t)>;
  using configField = MCP::Config::Field;

protected:
  REG reg_;
  MCP::MCP_MODEL model_;
  Config config_;
  PORT port_;
  bool bankMode_;
  uint8_t regAddress_;

  uint8_t enumIndex;
  bool readOnly = false;

  registerIdentity identity_;

  std::array<Callback, MAX_CALLBACK_PER_REG> callbacks_;
  union {
    uint8_t value; // Full register value
    struct {
      uint8_t bit0 : 1;
      uint8_t bit1 : 1;
      uint8_t bit2 : 1;
      uint8_t bit3 : 1;
      uint8_t bit4 : 1;
      uint8_t bit5 : 1;
      uint8_t bit6 : 1;
      uint8_t bit7 : 1;
    };
  };

public:
  RegisterBase() { callbacks_.fill(nullptr); }
  virtual ~RegisterBase() = default;
  enum Field : uint8_t {
    BIT_0 = 0,
    BIT_1,
    BIT_2,
    BIT_3,
    BIT_4,
    BIT_5,
    BIT_6,
    BIT_7,
  };

  virtual void setModel(MCP::MCP_MODEL m) = 0;
  virtual void updateRegisterAddress() = 0;
  virtual void setRegEnum(MCP::REG reg) = 0;
  virtual void setAddress(uint8_t address) = 0;
  virtual uint8_t getAddress() const = 0;

  virtual void initialiseValue() {

    if (reg_ == REG::IOCON) {
      value = 0X00;
      config_.configureDefault();
    } else if (reg_ == REG::IODIR) {

      value = 0XFF;
    } else {
      value = 0X00;
    }
  }

  virtual void setReadonly() { readOnly = true; }

  void useDefaultConfiguration() { return config_.configureDefault(); }

  uint8_t getSavedValue() const {

    return reg_ == REG::IOCON ? config_.getSettingValue() : value;
  }
  uint8_t getSavedSettings() const { return config_.getSettingValue(); }
  void updateState(currentEvent &ev) {
    if (ev.regIdentity.regAddress == regAddress_) {
      setValue(ev.data);
      if (reg_ == REG::IOCON) {
        config_.configure(ev.data);
      }
    }
  }
  bool updateState(uint8_t newValue) {
    if (reg_ == REG::IOCON) {
      return config_.configure(newValue);
    } else {
      value = newValue;
      return true;
    }
  }

  bool removeCallback(int index) {
    if (index >= 0 && index < static_cast<int>(callbacks_.size())) {
      callbacks_[index] = nullptr;
      return true;
    }
    return false; // Invalid index
  }
  void invokeCallbacks(uint8_t value) {
    for (const auto &cb : callbacks_) {
      if (cb) {
        cb(value);
      }
    }
  }

  virtual void applyMask(uint8_t mask) {

    if (reg_ != REG::IOCON) {
      value |= mask;
      EventManager::createEvent(identity_, RegisterEvent::WRITE_REQUEST, value);
    }
  }

  virtual void clearMask(uint8_t mask) {
    if (reg_ != REG::IOCON) {
      value &= ~mask;
      EventManager::createEvent(identity_, RegisterEvent::WRITE_REQUEST, value);
    }
  }

  bool setValue(uint8_t newValue) {
    if (reg_ == REG::INTF) {
      return false;
    } else if (reg_ == REG::IOCON) {

      bool success = config_.configure(newValue);
      if (success) {
        EventManager::createEvent(identity_, RegisterEvent::SETTINGS_CHANGED,
                                  newValue);
      }

      return success;

    } else {
      value = newValue;
      EventManager::createEvent(identity_, RegisterEvent::WRITE_REQUEST,
                                newValue);
      return true;
    }
  }

  void setBitField(Field field, bool value) {

    if (readOnly || (reg_ == REG::IOCON) || (reg_ == REG::INTF)) {
      ESP_LOGW("RegisterBase",
               "Attempted to modify a read-only or unsupported register");
      return;
    }

    if (value) {
      this->value |= (1 << static_cast<uint8_t>(field));
    } else {
      this->value &= ~(1 << static_cast<uint8_t>(field));
    }

    EventManager::createEvent(identity_, RegisterEvent::WRITE_REQUEST,
                              this->value);

    ESP_LOGI("RegisterBase", "Bit field %d set to %d in register 0x%02X", field,
             value, regAddress_);
  }

  bool getBitField(Field field) const {
    EventManager::createEvent(identity_, RegisterEvent::READ_REQUEST);

    EventBits_t bits = xEventGroupWaitBits(
        EventManager::registerEventGroup,
        static_cast<EventBits_t>(RegisterEvent::DATA_RECEIVED), pdFALSE,
        pdFALSE, READ_TIMEOUT);

    if (!(bits & static_cast<EventBits_t>(RegisterEvent::DATA_RECEIVED))) {
      ESP_LOGE(REG_TAG, "Timeout waiting for DATA_RECEIVED event!");
      return false;
    }
    currentEvent *event = EventManager::getEvent(RegisterEvent::DATA_RECEIVED);
    EventManager::acknowledgeEvent(event);
    EventManager::clearBits(RegisterEvent::DATA_RECEIVED);

    return (this->value & (1 << static_cast<uint8_t>(field))) != 0;
  }

  bool getBitField(configField field) const {
    EventManager::createEvent(identity_, RegisterEvent::READ_REQUEST);

    // Wait for the DATA_RECEIVED event
    EventBits_t bits = xEventGroupWaitBits(
        EventManager::registerEventGroup,
        static_cast<EventBits_t>(RegisterEvent::DATA_RECEIVED), pdFALSE,
        pdFALSE, READ_TIMEOUT);

    if (!(bits & static_cast<EventBits_t>(RegisterEvent::DATA_RECEIVED))) {
      ESP_LOGE(REG_TAG, "Timeout waiting for DATA_RECEIVED event!");
      return false;
    }
    currentEvent *event = EventManager::getEvent(RegisterEvent::DATA_RECEIVED);
    EventManager::acknowledgeEvent(event);
    EventManager::clearBits(RegisterEvent::DATA_RECEIVED);
    return config_.getBitField(field);
  }

  uint8_t getValue() const {
    EventManager::createEvent(identity_, RegisterEvent::READ_REQUEST);

    EventBits_t bits = xEventGroupWaitBits(
        EventManager::registerEventGroup,
        static_cast<EventBits_t>(RegisterEvent::DATA_RECEIVED), pdFALSE,
        pdFALSE, READ_TIMEOUT);

    if (!(bits & static_cast<EventBits_t>(RegisterEvent::DATA_RECEIVED))) {
      ESP_LOGE(REG_TAG, "Timeout waiting for DATA_RECEIVED event!");
      return 0xFF;
    }
    currentEvent *event = EventManager::getEvent(RegisterEvent::DATA_RECEIVED);
    EventManager::acknowledgeEvent(event);
    EventManager::clearBits(RegisterEvent::DATA_RECEIVED);
    return (reg_ == REG::IOCON) ? config_.getSettingValue() : value;
  }

  // Add a callback
  int addCallback(const Callback &callback) {
    for (size_t i = 0; i < callbacks_.size(); ++i) {
      if (!callbacks_[i]) {

        callbacks_[i] = callback;
        return static_cast<int>(i);
      }
    }
    return -1; // No space available
  }

  // IOCON-specific methods

  template <REG T>
  typename std::enable_if<T == REG::IOCON, bool>::type
  configure(const uint8_t &settings) {
    return config_.configure(settings);
  };
  template <REG T>
  typename std::enable_if<T == REG::IOCON, bool>::type
  configure(const Settings &setting) {
    return config_.configure(setting);
  }

  template <REG T>
  typename std::enable_if<T == REG::IOCON, void>::type
  setOperationMode(bool byteMode, bool mapping8Bit) {
    config_.setOperationMode(byteMode, mapping8Bit);
    if (mapping8Bit) {
      bankMode_ = true;

      updateRegisterAddress();
      EventManager::createEvent(identity_, RegisterEvent::BANK_MODE_CHANGED,
                                config_.getSettingValue());
    } else {
      EventManager::createEvent(identity_, RegisterEvent::SETTINGS_CHANGED,
                                config_.getSettingValue());
    }
  }

  template <REG T>
  typename std::enable_if<T == REG::IOCON, void>::type
  setInterruptSahring(bool enable) {
    config_.setMirror(enable);
    EventManager::createEvent(identity_, RegisterEvent::SETTINGS_CHANGED,
                              config_.getSettingValue());
  }

  template <REG T>
  typename std::enable_if<T == REG::IOCON, void>::type
  setSlewRate(bool disable_state) {
    config_.setSlewRate(disable_state);
    EventManager::createEvent(identity_, RegisterEvent::SETTINGS_CHANGED,
                              config_.getSettingValue());
  }

  template <REG T>
  typename std::enable_if<T == REG::IOCON, bool>::type
  setHardwareAddressing(bool enable) {
    if (model_ == MCP::MCP_MODEL::MCP23S17) {
      config_.setHardwareAddressing(enable);
      EventManager::createEvent(identity_, RegisterEvent::SETTINGS_CHANGED,
                                config_.getSettingValue());
      return true;
    }
    return false;
  }

  template <REG T>
  typename std::enable_if<T == REG::IOCON, bool>::type
  setOpenDrain(bool enable) {
    bool intPolMode = config_.getBitField(configField::INTPOL);

    if (!intPolMode && enable) {
      config_.setOpenDrain(true);
      config_.setInterruptPolarity(false);
      EventManager::createEvent(identity_, RegisterEvent::SETTINGS_CHANGED,
                                config_.getSettingValue());
      return true;
    } else if (intPolMode && enable) {
      config_.setOpenDrain(false);
      EventManager::createEvent(identity_, RegisterEvent::SETTINGS_CHANGED,
                                config_.getSettingValue());
      return true;
    }
    return false;
  }

  template <REG T>
  typename std::enable_if<T == REG::IOCON, bool>::type
  setInterruptPolarity(bool activeHigh) {
    bool outputMode = config_.getBitField(configField::ODR);
    if (!outputMode) {
      config_.setInterruptPolarity(activeHigh);
      EventManager::createEvent(identity_, RegisterEvent::SETTINGS_CHANGED,
                                config_.getSettingValue());
      return true;
    }
    return false;
  }

  template <REG T>
  typename std::enable_if<T == REG::IOCON, bool>::type getBankMode() const {
    return config_.getBitField(configField::BANK);
  }
  template <REG T>
  typename std::enable_if<T == REG::IOCON, bool>::type
  getSequentialMode() const {
    return config_.getBitField(configField::SEQOP);
  }

  template <REG T>
  typename std::enable_if<T == REG::IOCON, uint8_t>::type
  getSettingValue() const {
    return config_.getSettingValue();
  }

  template <REG T>
  typename std::enable_if<T == REG::INTF, uint8_t>::type
  getInterruptFlag() const {
    return getValue();
  }
};

//

class MCPRegister : public RegisterBase {

public:
  MCPRegister(MCP::MCP_MODEL m, REG rg, PORT p, bool bm) {
    setModel(m);
    setRegEnum(rg);
    config_ = Config(m);
    port_ = p;
    bankMode_ = bm;
    regAddress_ = calculateAddress(rg, p);

    identity_ = registerIdentity(rg, p, regAddress_);
    initialiseValue();
  }

  void setModel(MCP::MCP_MODEL m) override { model_ = m; }

  void setRegEnum(MCP::REG reg) override {
    reg_ = reg;
    updateRegisterAddress();
  }
  bool updatebankMode(bool mode) {
    bankMode_ = mode;
    return true;
  }
  void updateRegisterAddress() override {
    uint8_t newAddress = calculateAddress(reg_, port_);
    setAddress(newAddress);
    identity_.regAddress = newAddress;
  }
  void setAddress(uint8_t address) override { regAddress_ = address; }

  uint8_t getAddress() const override { return regAddress_; }

  template <REG T>
  typename std::enable_if<T == REG::GPIO, bool>::type //
  readPin(PIN pin) {
    if (port_ != Util::getPortFromPin(pin)) {
      ESP_LOGE(REG_TAG, "Pinmask does not belong to the expected port");
      return false;
    }
    uint8_t index = Util::getPinIndex(pin);
    return getBitField(static_cast<Field>(index));
  }
  template <REG T>
  typename std::enable_if<T == REG::GPIO, uint8_t>::type //
  readPins() {

    return getValue();
  }
  template <REG T>
  typename std::enable_if<T == REG::GPIO, uint8_t>::type //
  readPins(uint8_t pinmask) {

    return getValue() & pinmask;
  }

  template <REG T>
  typename std::enable_if<T == REG::IODIR, bool>::type
  setPinMode(PIN pin, GPIO_MODE mode) {

    if (port_ != Util::getPortFromPin(pin)) {
      ESP_LOGE(REG_TAG, "Pin %d does not belong to the expected port",
               static_cast<int>(pin));
      return false;
    }
    uint8_t index = Util::getPinIndex(pin);
    setBitField(static_cast<Field>(index), mode == GPIO_MODE::GPIO_INPUT);

    return true;
  }
  template <REG T>
  typename std::enable_if<T == REG::IODIR, bool>::type
  setPinMode(uint8_t pinmask, GPIO_MODE mode) {

    if (mode == GPIO_MODE::GPIO_INPUT) {
      applyMask(pinmask);
    } else {
      clearMask(pinmask);
    }

    return true;
  }

  template <REG T>
  typename std::enable_if<T == REG::GPPU, bool>::type
  setPullType(PIN pin, PULL_MODE mode) {

    if (port_ != Util::getPortFromPin(pin)) {
      ESP_LOGE(REG_TAG, "Pin %d does not belong to the expected port",
               static_cast<int>(pin));
      return false;
    }
    uint8_t index = Util::getPinIndex(pin);
    setBitField(static_cast<Field>(index), mode == PULL_MODE::ENABLE_PULLUP);

    return true;
  }
  template <REG T>
  typename std::enable_if<T == REG::GPPU, bool>::type
  setPullType(uint8_t pinmask, PULL_MODE mode) {

    if (mode == PULL_MODE::ENABLE_PULLUP) {
      applyMask(pinmask);
    } else {
      clearMask(pinmask);
    }

    return true;
  }
  template <REG T>
  typename std::enable_if<T == REG::IPOL, bool>::type
  setInputPolarity(PIN pin, INPUT_POLARITY pol) {

    if (port_ != Util::getPortFromPin(pin)) {
      ESP_LOGE(REG_TAG, "Pin %d does not belong to the expected port",
               static_cast<int>(pin));
      return false;
    }
    uint8_t index = Util::getPinIndex(pin);
    setBitField(static_cast<Field>(index), pol == INPUT_POLARITY::INVERTED);

    return true;
  }
  template <REG T>
  typename std::enable_if<T == REG::IPOL, bool>::type
  setInputPolarity(uint8_t pinmask, INPUT_POLARITY pol) {

    if (pol == INPUT_POLARITY::INVERTED) {
      applyMask(pinmask);
    } else {
      clearMask(pinmask);
    }
    return true;
  }
  template <REG T>
  typename std::enable_if<T == REG::OLAT, bool>::type //
  setOutputLatch(PIN pin, bool lat) {

    if (port_ != Util::getPortFromPin(pin)) {
      ESP_LOGE(REG_TAG, "Pin %d does not belong to the expected port",
               static_cast<int>(pin));
      return false;
    }
    uint8_t index = Util::getPinIndex(pin);
    setBitField(static_cast<Field>(index), lat);

    return true;
  }
  template <REG T>
  typename std::enable_if<T == REG::OLAT, bool>::type
  setOutputLatch(uint8_t pinmask, bool lat) {

    if (lat) {
      applyMask(pinmask);
    } else {
      clearMask(pinmask);
    }
    ESP_LOGI("RegisterBase",
             "Bits 0x%02X are set with value %d in register 0x%02X", pinmask,
             getSavedValue(), regAddress_);
    return true;
  }

  // INTERRUPT SPECIFIC
  template <REG T>
  typename std::enable_if<T == REG::GPINTEN, bool>::type
  setInterruptOnChange(PIN pin, INTR_ON_CHANGE_ENABLE en) {

    if (port_ != Util::getPortFromPin(pin)) {
      ESP_LOGE(REG_TAG, "Pin %d does not belong to the expected port",
               static_cast<int>(pin));
      return false;
    }
    uint8_t index = Util::getPinIndex(pin);
    setBitField(static_cast<Field>(index),
                en == INTR_ON_CHANGE_ENABLE::ENABLE_INTR_ON_CHANGE);

    return true;
  }
  template <REG T>
  typename std::enable_if<T == REG::GPINTEN, bool>::type
  setInterruptOnChange(uint8_t pinmask, INTR_ON_CHANGE_ENABLE en) {

    if (en == INTR_ON_CHANGE_ENABLE::ENABLE_INTR_ON_CHANGE) {
      applyMask(pinmask);
    } else {
      clearMask(pinmask);
    }
    return true;
  }
  template <REG T>
  typename std::enable_if<T == REG::INTCON, bool>::type
  setInterruptType(PIN pin, INTR_ON_CHANGE_CONTROL cntrl) {

    if (port_ != Util::getPortFromPin(pin)) {
      ESP_LOGE(REG_TAG, "Pin %d does not belong to the expected port",
               static_cast<int>(pin));
      return false;
    }
    uint8_t index = Util::getPinIndex(pin);
    setBitField(static_cast<Field>(index),
                cntrl == INTR_ON_CHANGE_CONTROL::COMPARE_WITH_DEFVAL);

    return true;
  }
  template <REG T>
  typename std::enable_if<T == REG::INTCON, bool>::type
  setInterruptType(uint8_t pinmask, INTR_ON_CHANGE_CONTROL cntrl) {

    if (cntrl == INTR_ON_CHANGE_CONTROL::COMPARE_WITH_DEFVAL) {
      applyMask(pinmask);
    } else {
      clearMask(pinmask);
    }

    return true;
  }

  template <REG T>
  typename std::enable_if<T == REG::DEFVAL, bool>::type
  saveCompareValue(PIN pin, DEF_VAL_COMPARE cmp) {
    if (port_ != Util::getPortFromPin(pin)) {
      ESP_LOGE(REG_TAG, "Pinmask does not belong to the expected port");
      return false;
    }
    uint8_t index = Util::getPinIndex(pin);
    setBitField(static_cast<Field>(index),
                cmp == DEF_VAL_COMPARE::SAVE_LOGIC_HIGH);

    return true;
  }
  template <REG T>
  typename std::enable_if<T == REG::DEFVAL, bool>::type
  saveCompareValue(uint8_t pinmask, DEF_VAL_COMPARE cmp) {

    if (cmp == DEF_VAL_COMPARE::SAVE_LOGIC_HIGH) {
      applyMask(pinmask);
    } else {
      clearMask(pinmask);
    }

    return true;
  }
  template <REG T>
  typename std::enable_if<T == REG::INTCAP, bool>::type clearInterrupt() {

    if (getValue()) {
      return true;
    } else {
      return false;
    }
  }

private:
  constexpr uint8_t calculateAddress(REG reg, PORT port) const {
    uint8_t baseAddress = static_cast<uint8_t>(reg);

    if (bankMode_) {
      // BANK = 1: Separate PORTA and PORTB
      return baseAddress + (port == PORT::GPIOB ? 0x10 : 0x00);
    } else {
      // BANK = 0: Paired A/B registers
      return (baseAddress * 2) + (port == PORT::GPIOB ? 0x01 : 0x00);
    }

    return baseAddress;
  }
};
class MCPRegisters {
private:
  std::unordered_map<MCP::REG, std::unique_ptr<MCP::MCPRegister>> regMap;

public:
  std::shared_ptr<MCP::MCPRegister> iocon;
  // Raw pointers for easier access
  MCP::MCPRegister *iodir{};
  MCP::MCPRegister *gppu{};
  MCP::MCPRegister *ipol{};
  MCP::MCPRegister *gpio{};
  MCP::MCPRegister *olat{};
  MCP::MCPRegister *gpinten{};
  MCP::MCPRegister *intcon{};
  MCP::MCPRegister *defval{};
  MCP::MCPRegister *intf{};
  MCP::MCPRegister *intcap{};

  MCPRegisters() = default;

  void setup(MCP::MCP_MODEL model, PORT port, bool bankMode) {
    iocon = std::make_shared<MCP::MCPRegister>(model, MCP::REG::IOCON, port,
                                               bankMode);

    for (auto regType :
         {MCP::REG::IODIR, MCP::REG::GPPU, MCP::REG::IPOL, MCP::REG::GPIO,
          MCP::REG::OLAT, MCP::REG::GPINTEN, MCP::REG::INTCON, MCP::REG::DEFVAL,
          MCP::REG::INTF, MCP::REG::INTCAP}) {
      regMap[regType] =
          std::make_unique<MCP::MCPRegister>(model, regType, port, bankMode);
    }

    assign(); // Assign raw pointers
  }

  void assign() {
    iodir = regMap[MCP::REG::IODIR].get();
    gppu = regMap[MCP::REG::GPPU].get();
    ipol = regMap[MCP::REG::IPOL].get();
    gpio = regMap[MCP::REG::GPIO].get();
    olat = regMap[MCP::REG::OLAT].get();
    gpinten = regMap[MCP::REG::GPINTEN].get();
    intcon = regMap[MCP::REG::INTCON].get();
    defval = regMap[MCP::REG::DEFVAL].get();
    intf = regMap[MCP::REG::INTF].get();
    intcap = regMap[MCP::REG::INTCAP].get();
  }
  void updateAddress(bool bankMode) {

    if (iocon) {
      iocon->updatebankMode(bankMode);
      iocon->updateRegisterAddress();
    }

    for (auto &[regType, regPtr] : regMap) {
      regPtr->updatebankMode(bankMode);
      regPtr->updateRegisterAddress();
    }
  }

  std::shared_ptr<MCP::MCPRegister> const getIOCON() { return iocon; }

  // ✅ Getter for read-only access
  const MCP::MCPRegister *getRegister(MCP::REG regType) const {
    if (regType == MCP::REG::IOCON)
      return iocon.get();
    auto it = regMap.find(regType);
    return (it != regMap.end()) ? it->second.get() : nullptr;
  }

  // ✅ Getter for modifying the register
  MCP::MCPRegister *getRegisterForUpdate(MCP::REG regType) {
    if (regType == MCP::REG::IOCON)
      return iocon.get();
    auto it = regMap.find(regType);
    return (it != regMap.end()) ? it->second.get() : nullptr;
  }
  const MCP::MCPRegister *getRegister(uint8_t address) const {

    for (auto &[regType, regPtr] : regMap) {
      if (regPtr->getAddress() == address) {
        return regPtr.get();
      }
    }
    return nullptr;
  }

  uint8_t getSavedValue(MCP::REG reg) const {
    if (reg == MCP::REG::IOCON) {
      return iocon->getSavedValue();
    } else {
      auto it = regMap.find(reg);
      return it != regMap.end() ? it->second->getSavedValue() : 0;
    }
  }

  uint8_t getAddress(MCP::REG reg) const {
    if (reg == MCP::REG::IOCON) {
      return iocon->getAddress();
    } else {
      auto it = regMap.find(reg);
      return it != regMap.end() ? it->second->getAddress() : 0;
    }
  }
  bool updateRegisterValue(uint8_t reg_address, uint8_t value) {
    if (iocon->getAddress() == reg_address) {
      return iocon->updateState(value);
    }

    for (auto &[regType, regPtr] : regMap) {
      if (regPtr->getAddress() == reg_address) {
        return regPtr->updateState(value);
      }
    }

    ESP_LOGE("MCPRegisters",
             "Failed to update register: Invalid address (0x%02X)",
             reg_address);
    return false;
  }
};

struct Register {
  const MCP::REG reg;
  const MCP::PORT port;
  Register(MCP::REG rg = MCP::REG::IODIR, MCP::PORT p = MCP::PORT::GPIOA,
           bool readonly = false)
      : reg(rg), port(p), value_(0),
        regAddress_(Util::calculateAddress(reg, port, bankSeparated_)),
        identity_(reg, port, regAddress_), readOnly_(readonly) {}

  void updateBankMode(bool bankMode) {
    bankSeparated_ = bankMode;
    regAddress_ = Util::calculateAddress(reg, port, bankSeparated_);
    identity_.regAddress = regAddress_;
  }
  void setValue(uint8_t newvalue) {
    if (!readOnly_) {

      value_ = newvalue;
      if (reg == MCP::REG::IOCON) {
        EventManager::createEvent(identity_, RegisterEvent::SETTINGS_CHANGED,
                                  value_);
      } else {
        EventManager::createEvent(identity_, RegisterEvent::WRITE_REQUEST,
                                  value_);
      }
    }
  }

  void setBitField(uint8_t bit, bool bit_value) {

    if (!readOnly_) {
      if (bit < 8) {
        if (bit_value) {
          Util::BIT::set(value_, bit);
        } else {
          Util::BIT::clear(value_, bit);
        }
      }
      if (reg == MCP::REG::IOCON) {
        EventManager::createEvent(identity_, RegisterEvent::SETTINGS_CHANGED,
                                  value_);
      } else {
        EventManager::createEvent(identity_, RegisterEvent::WRITE_REQUEST,
                                  value_);
      }
    }
  }
  uint8_t getValue() {

    EventManager::createEvent(identity_, RegisterEvent::READ_REQUEST);

    EventBits_t bits = xEventGroupWaitBits(
        EventManager::registerEventGroup,
        static_cast<EventBits_t>(RegisterEvent::DATA_RECEIVED), pdFALSE,
        pdFALSE, READ_TIMEOUT);

    if (!(bits & static_cast<EventBits_t>(RegisterEvent::DATA_RECEIVED))) {
      ESP_LOGE(REG_TAG, "Timeout waiting for DATA_RECEIVED event!");
      return 0xFF;
    }
    currentEvent *event = EventManager::getEvent(RegisterEvent::DATA_RECEIVED);
    EventManager::acknowledgeEvent(event);
    EventManager::clearBits(RegisterEvent::DATA_RECEIVED);

    return value_;
  }
  bool getBitField(uint8_t bit) {
    EventManager::createEvent(identity_, RegisterEvent::READ_REQUEST);

    EventBits_t bits = xEventGroupWaitBits(
        EventManager::registerEventGroup,
        static_cast<EventBits_t>(RegisterEvent::DATA_RECEIVED), pdFALSE,
        pdFALSE, READ_TIMEOUT);

    if (!(bits & static_cast<EventBits_t>(RegisterEvent::DATA_RECEIVED))) {
      ESP_LOGE(REG_TAG, "Timeout waiting for DATA_RECEIVED event!");
      return 0xFF;
    }
    currentEvent *event = EventManager::getEvent(RegisterEvent::DATA_RECEIVED);
    EventManager::acknowledgeEvent(event);
    EventManager::clearBits(RegisterEvent::DATA_RECEIVED);
    return (bit < 8) ? Util::BIT::isSet(value_, bit) : false;
  }
  void applyMask(uint8_t mask) {

    if (reg != REG::IOCON) {
      value_ |= mask;
      EventManager::createEvent(identity_, RegisterEvent::WRITE_REQUEST,
                                value_);
    }
  }

  void clearMask(uint8_t mask) {
    if (reg != REG::IOCON) {
      value_ &= ~mask;
      EventManager::createEvent(identity_, RegisterEvent::WRITE_REQUEST,
                                value_);
    }
  }
  uint8_t getAddress() const { return regAddress_; }
  registerIdentity getIdentity() const { return identity_; }

private:
  uint8_t value_;
  uint8_t regAddress_;
  registerIdentity identity_;
  bool readOnly_ = false;
  bool bankSeparated_ = false;
};

} // namespace MCP
#endif
