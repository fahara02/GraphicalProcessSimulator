#ifndef MCP_REGISTERS_HPP
#define MCP_REGISTERS_HPP
#include "MCP_Constants.hpp"
#include "MCP_Primitives.hpp"
#include "RegisterEvents.hpp"
#include <functional>
#include <memory>
#include <type_traits>

#define REG_TAG "MCP_REGISTERS"
namespace MCP {

struct config_icon_t {
public:
public:
  // Define fields as a public enum for easier reference
  enum Field : uint8_t {
    BANK = 0,
    MIRROR,
    SEQOP,
    DISSLW,
    HAEN,
    ODR,
    INTPOL,
    RESERVED
  };

private:
  union {
    uint8_t value; // Full register value
    struct {
      uint8_t bank : 1;     //!< Controls how the registers are addressed
      uint8_t mirror : 1;   //!< INT Pins Mirror bit
      uint8_t seqop : 1;    //!< Sequential Operation mode bit
      uint8_t disslw : 1;   //!< Slew Rate control bit for SDA output
      uint8_t haen : 1;     //!< Enables hardware addressing
      uint8_t odr : 1;      //!< Configures the INT pin as an open-drain output
      uint8_t intpol : 1;   //!< Sets the polarity of the INT output pin
      uint8_t reserved : 1; //!< Reserved bit (unused)
    };
  };

public:
  config_icon_t() : value(0) {}
  config_icon_t(uint8_t setting)
      : value(configure(setting) == true ? configure(setting) : 0) {}

  void setBitField(Field field, bool value) {
    uint8_t *fields = reinterpret_cast<uint8_t *>(this);
    if (value)
      *fields |= (1 << field);
    else
      *fields &= ~(1 << field);
  }

  bool getBitField(Field field) const {
    const uint8_t *fields = reinterpret_cast<const uint8_t *>(this);
    return (*fields & (1 << field)) != 0;
  }
  uint8_t getSettings() const { return value; }
  bool configure(uint8_t newSettings) {

    if (newSettings & (1 << Field::RESERVED)) {
      return false;
    }

    bool intPol = (newSettings & (1 << Field::INTPOL)) != 0;
    bool odr = (newSettings & (1 << Field::ODR)) != 0;

    if (intPol && odr) {
      return false;
    }

    value = newSettings;
    return true;
  }
  void applyMask(uint8_t mask) {
    { value |= mask; }
  }

  void clearMask(uint8_t mask) { value &= ~mask; }
};

//
class ioconBase {
protected:
  uint8_t address[MAX_REG_PER_DEVICE];

public:
  virtual ~ioconBase() = default;
  using Field = MCP::config_icon_t::Field;

  virtual void updateRegisters() = 0;

  virtual void setCallBack(const std::function<void()> &cb) = 0;

  virtual bool configure(const uint8_t &settings) {
    bool status = settings_.configure(settings);
    if (status) {
      settings_ = config_icon_t(settings);
    };
    return status;
  };

  virtual void setModel(MCP::MCP_MODEL model) { model_ = model; }

  // setting methods with safe enums
  virtual void setBankMode(MCP_BANK_MODE mode) {
    settings_.setBitField(Field::BANK, mode == MCP_BANK_MODE::SEPARATE_BANK);
    updateRegisters();
  }

  virtual void setInterruptMode(MCP_MIRROR_MODE mode) {
    settings_.setBitField(Field::MIRROR,
                          mode == MCP_MIRROR_MODE::INT_CONNECTED);
  }

  virtual void setOperationMode(MCP_OPERATION_MODE mode) {
    settings_.setBitField(Field::SEQOP, mode == MCP_OPERATION_MODE::BYTE_MODE);
  }

  virtual void setSlewRateMode(MCP_SLEW_RATE mode) {
    settings_.setBitField(Field::DISSLW, mode == MCP_SLEW_RATE::SLEW_DISABLED);
  }

  virtual bool setHardwareAddressingMode(MCP_HARDWARE_ADDRESSING mode) {
    if (model_ == MCP::MCP_MODEL::MCP23S17) {
      settings_.setBitField(Field::HAEN,
                            mode == MCP_HARDWARE_ADDRESSING::HAEN_ENABLED);
      return true;
    }
    return false;
  }
  virtual bool setOutputMode(MCP_OPEN_DRAIN mode) {
    bool IntpolMode = settings_.getBitField(Field::INTPOL);

    if (!IntpolMode && mode == MCP_OPEN_DRAIN::ODR) {
      settings_.setBitField(Field::ODR, true);
      settings_.setBitField(Field::INTPOL, false);
      return true;
    } else if (IntpolMode && mode == MCP_OPEN_DRAIN::ACTIVE_DRIVER) {
      settings_.setBitField(Field::ODR, false);
      return true;
    }
    return false;
  }
  virtual bool setInterruptPolarityMode(MCP_INT_POL mode) {
    bool outputMode = settings_.getBitField(Field::ODR);
    if (!outputMode) {
      settings_.setBitField(Field::INTPOL,
                            mode == MCP_INT_POL::MCP_ACTIVE_HIGH);
      return true;
    }
    return false;
  }
  // direct setting method
  virtual void mergeBanks() {
    settings_.setBitField(Field::BANK, true);
    updateRegisters();
  }
  virtual void separateBanks() {
    settings_.setBitField(Field::BANK, false);
    updateRegisters();
  }
  virtual void mergeInterrupts() { settings_.setBitField(Field::MIRROR, true); }
  virtual void separateInterrupts() {
    settings_.setBitField(Field::MIRROR, false);
  }

  virtual void enableContinousPoll() {
    settings_.setBitField(Field::SEQOP, false);
  }
  virtual void disableContinousPoll() {
    settings_.setBitField(Field::SEQOP, true);
  }
  virtual void enableSlewRate() { settings_.setBitField(Field::DISSLW, false); }
  virtual void disableSlewRate() { settings_.setBitField(Field::DISSLW, true); }
  virtual void enableOpenDrain() {
    settings_.setBitField(Field::ODR, true);
    disableIntrruptActiveHigh();
  }
  virtual void disableOpenDrain() { settings_.setBitField(Field::ODR, false); }
  virtual void enableIntrruptActiveHigh() {
    settings_.setBitField(Field::INTPOL, true);
    disableOpenDrain();
  }
  virtual void disableIntrruptActiveHigh() {
    settings_.setBitField(Field::INTPOL, false);
  }

  virtual bool getBankMode() const {
    return settings_.getBitField(Field::BANK);
  }

  virtual uint8_t getSettings() const { return settings_.getSettings(); }

protected:
  config_icon_t settings_;
  MCP::MCP_MODEL model_;
};

//
class ControlRegister : public ioconBase {
private:
  std::function<void()> callback;

public:
  ControlRegister(MCP::MCP_MODEL m) { setModel(m); }

  void setCallBack(const std::function<void()> &cb) override { callback = cb; }
  void updateRegisters() override {
    if (callback) {
      callback();
    }
  }
};

//
class RegisterBase {
protected:
  REG reg_;
  MCP::MCP_MODEL model_;
  config_icon_t settings_;
  uint8_t enumIndex;
  bool readOnly = false;

public:
  using Callback = std::function<void(uint8_t)>;
  using configField = MCP::config_icon_t::Field;
  RegisterBase() { callbacks_.fill(nullptr); }
  virtual ~RegisterBase() = default;
  enum Field : uint8_t {
    BIT_7 = 0,
    BIT_6,
    BIT_5,
    BIT_4,
    BIT_3,
    BIT_2,
    BIT_1,
    BIT_0,
  };

  virtual void setModel(MCP::MCP_MODEL m) = 0;
  virtual void updateRegisterAddress() = 0;
  virtual void setRegEnum(MCP::REG reg) = 0;
  virtual void setAddress(uint8_t address) = 0;
  virtual uint8_t getAddress() const = 0;

  virtual void setReadonly() { readOnly = true; }

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

    if (reg_ == REG::IOCON) {
      return settings_.applyMask(mask);
    } else {
      value |= mask;
    }
    EventManager::setBits(RegisterEvent::WRITE_REQUEST);
  }

  virtual void clearMask(uint8_t mask) {
    if (reg_ == REG::IOCON) {
      return settings_.clearMask(mask);
    } else {
      value &= ~mask;
    }
    EventManager::setBits(RegisterEvent::WRITE_REQUEST);
  }

  bool setValue(uint8_t newValue) {
    if (reg_ == REG::INTF) {
      return false;
    } else if (reg_ == REG::IOCON) {

      EventManager::setBits(RegisterEvent::SETTINGS_CHANGED);
      return settings_.configure(newValue);

    } else {
      value = newValue;
      EventManager::setBits(RegisterEvent::WRITE_REQUEST);
      return true;
    }
  }

  void setBitField(Field field, bool value) {

    if ((reg_ == REG::IOCON) || (reg_ == REG::INTF)) {
      return;
    } else {
      uint8_t *fields = reinterpret_cast<uint8_t *>(this);
      if (value)
        *fields |= (1 << field);
      else
        *fields &= ~(1 << field);
      EventManager::setBits(RegisterEvent::WRITE_REQUEST);

      return;
    }
  }

  bool getBitField(Field field) const {
    EventManager::setBits(RegisterEvent::READ_REQUEST);
    const uint8_t *fields = reinterpret_cast<const uint8_t *>(this);
    return (*fields & (1 << field)) != 0;
  }
  bool getBitField(configField field) const {

    const uint8_t *fields = reinterpret_cast<const uint8_t *>(this);
    return (*fields & (1 << field)) != 0;
  }
  uint8_t getValue() const {
    if (reg_ == REG::IOCON) {
      return settings_.getSettings();
    } else {
      return value;
    }
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
  typename std::enable_if<T == REG::IOCON, void>::type
  setBankMode(MCP_BANK_MODE mode) {
    settings_.setBitField(configField::BANK,
                          mode == MCP_BANK_MODE::SEPARATE_BANK);
    updateRegisterAddress();
  }

  template <REG T>
  typename std::enable_if<T == REG::IOCON, void>::type
  setInterruptSahring(MCP_MIRROR_MODE mode) {
    settings_.setBitField(configField::MIRROR,
                          mode == MCP_MIRROR_MODE::INT_CONNECTED);
  }

  template <REG T>
  typename std::enable_if<T == REG::IOCON, void>::type
  setOperationMode(MCP_OPERATION_MODE mode) {
    settings_.setBitField(configField::SEQOP,
                          mode == MCP_OPERATION_MODE::BYTE_MODE);
  }

  template <REG T>
  typename std::enable_if<T == REG::IOCON, void>::type
  setSlewRateMode(MCP_SLEW_RATE mode) {
    settings_.setBitField(configField::DISSLW,
                          mode == MCP_SLEW_RATE::SLEW_DISABLED);
  }

  template <REG T>
  typename std::enable_if<T == REG::IOCON, bool>::type
  setHardwareAddressing(MCP_HARDWARE_ADDRESSING mode) {
    if (model_ == MCP::MCP_MODEL::MCP23S17) {
      settings_.setBitField(configField::HAEN,
                            mode == MCP_HARDWARE_ADDRESSING::HAEN_ENABLED);
      return true;
    }
    return false;
  }

  template <REG T>
  typename std::enable_if<T == REG::IOCON, bool>::type
  setOpenDrain(MCP_OPEN_DRAIN mode) {
    bool intPolMode = settings_.getBitField(configField::INTPOL);

    if (!intPolMode && mode == MCP_OPEN_DRAIN::ODR) {
      settings_.setBitField(configField::ODR, true);
      settings_.setBitField(configField::INTPOL, false);
      return true;
    } else if (intPolMode && mode == MCP_OPEN_DRAIN::ACTIVE_DRIVER) {
      settings_.setBitField(configField::ODR, false);
      return true;
    }
    return false;
  }

  template <REG T>
  typename std::enable_if<T == REG::IOCON, bool>::type
  setInterruptPolarity(MCP_INT_POL mode) {
    bool outputMode = settings_.getBitField(configField::ODR);
    if (!outputMode) {
      settings_.setBitField(configField::INTPOL,
                            mode == MCP_INT_POL::MCP_ACTIVE_HIGH);
      return true;
    }
    return false;
  }

  template <REG T>
  typename std::enable_if<T == REG::IOCON, void>::type mergeBanks() {
    settings_.setBitField(configField::BANK, true);
    updateRegisterAddress();
  }

  template <REG T>
  typename std::enable_if<T == REG::IOCON, void>::type separateBanks() {
    settings_.setBitField(configField::BANK, false);
    updateRegisterAddress();
  }

  template <REG T>
  typename std::enable_if<T == REG::IOCON, void>::type mergeInterrupts() {
    settings_.setBitField(configField::MIRROR, true);
  }

  template <REG T>
  typename std::enable_if<T == REG::IOCON, void>::type separateInterrupts() {
    settings_.setBitField(configField::MIRROR, false);
  }

  template <REG T>
  typename std::enable_if<T == REG::IOCON, void>::type enableContinuousPoll() {
    settings_.setBitField(configField::SEQOP, false);
  }

  template <REG T>
  typename std::enable_if<T == REG::IOCON, void>::type disableContinuousPoll() {
    settings_.setBitField(configField::SEQOP, true);
  }

  template <REG T>
  typename std::enable_if<T == REG::IOCON, void>::type enableSlewRate() {
    settings_.setBitField(configField::DISSLW, false);
  }

  template <REG T>
  typename std::enable_if<T == REG::IOCON, void>::type disableSlewRate() {
    settings_.setBitField(configField::DISSLW, true);
  }

  template <REG T>
  typename std::enable_if<T == REG::IOCON, void>::type enableOpenDrain() {
    settings_.setBitField(configField::ODR, true);
    disableInterruptActiveHigh<T>();
  }

  template <REG T>
  typename std::enable_if<T == REG::IOCON, void>::type disableOpenDrain() {
    settings_.setBitField(configField::ODR, false);
  }

  template <REG T>
  typename std::enable_if<T == REG::IOCON, void>::type
  enableInterruptActiveHigh() {
    settings_.setBitField(configField::INTPOL, true);
    disableOpenDrain<T>();
  }

  template <REG T>
  typename std::enable_if<T == REG::IOCON, void>::type
  disableInterruptActiveHigh() {
    settings_.setBitField(configField::INTPOL, false);
  }

  template <REG T>
  typename std::enable_if<T == REG::IOCON, bool>::type getBankMode() const {
    return settings_.getBitField(configField::BANK);
  }

  template <REG T>
  typename std::enable_if<T == REG::IOCON, uint8_t>::type getSettings() const {
    return settings_.getSettings();
  }

private:
  std::array<Callback, MAX_CALLBACK_PER_REG> callbacks_;
  union {
    uint8_t value; // Full register value
    struct {
      uint8_t bit7 : 1;
      uint8_t bit6 : 1;
      uint8_t bit5 : 1;
      uint8_t bit4 : 1;
      uint8_t bit3 : 1;
      uint8_t bit2 : 1;
      uint8_t bit1 : 1;
      uint8_t bit0 : 1;
    };
  };
};

//

class MCPRegister : public RegisterBase {
private:
  PORT port_;
  bool bankMode_;
  uint8_t regAddress_;

public:
  MCPRegister(MCP::MCP_MODEL m, REG rg, PORT p, bool bm)
      : port_(p), bankMode_(bm), regAddress_(calculateAddress(rg, p)) {
    setRegEnum(rg);

    initialiseValue(rg);

    setModel(m);
  }
  void initialiseValue(REG r) {}
  void setRegEnum(MCP::REG reg) override {
    reg_ = reg;
    updateRegisterAddress();
  }
  void setModel(MCP::MCP_MODEL m) override { model_ = m; }
  void updateRegisterAddress() override {
    setAddress(calculateAddress(reg_, port_));
  }
  void setAddress(uint8_t address) override { regAddress_ = address; }

  uint8_t getAddress() const override { return regAddress_; }

  template <REG T>
  typename std::enable_if<T == REG::GPIO, bool>::type //
  readPin(PIN pin) {
    uint8_t index = getIndexFromPin(pin);
    if (reg_ == REG::INTCON) {
      return getBitField(static_cast<configField>(index));
    } else {
      return getBitField(static_cast<Field>(index));
    }
  }
  template <REG T>
  typename std::enable_if<T == REG::GPIO, uint8_t>::type //
  readPins() {

    if (reg_ == REG::INTCON) {
      return settings_.getSettings();
    } else {
      return getValue();
    }
  }
  template <REG T>
  typename std::enable_if<T == REG::GPIO, uint8_t>::type //
  readPins(uint8_t pinmask) {
    uint8_t result = 0;

    if (reg_ == REG::INTCON) {

      result = settings_.getSettings() & pinmask;
    } else {

      result = getValue() & pinmask;
    }

    return result;
  }

  template <REG T>
  typename std::enable_if<T == REG::IODIR, bool>::type
  setPinMode(PIN pin, GPIO_MODE mode) {

    if (port_ != getPortFromPin(pin)) {
      ESP_LOGE(REG_TAG, "Pin %d does not belong to the expected port",
               static_cast<int>(pin));
      return false;
    }
    uint8_t index = getIndexFromPin(pin);
    setBitField(static_cast<Field>(index), mode == GPIO_MODE::GPIO_INPUT);

    return true;
  }
  template <REG T>
  typename std::enable_if<T == REG::IODIR, bool>::type
  setPinMode(uint8_t pinmask, PORT port, GPIO_MODE mode) {

    if (port_ != port) {
      ESP_LOGE(REG_TAG, "Pinmask does not belong to the expected port");
      return false;
    }

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

    if (port_ != getPortFromPin(pin)) {
      ESP_LOGE(REG_TAG, "Pin %d does not belong to the expected port",
               static_cast<int>(pin));
      return false;
    }
    uint8_t index = getIndexFromPin(pin);
    setBitField(static_cast<Field>(index), mode == PULL_MODE::ENABLE_PULLUP);

    return true;
  }
  template <REG T>
  typename std::enable_if<T == REG::GPPU, bool>::type
  setPullType(uint8_t pinmask, PORT port, PULL_MODE mode) {

    if (port_ != port) {
      ESP_LOGE(REG_TAG, "Pinmask does not belong to the expected port");
      return false;
    }

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

    if (port_ != getPortFromPin(pin)) {
      ESP_LOGE(REG_TAG, "Pin %d does not belong to the expected port",
               static_cast<int>(pin));
      return false;
    }
    uint8_t index = getIndexFromPin(pin);
    setBitField(static_cast<Field>(index), pol == INPUT_POLARITY::INVERTED);

    return true;
  }
  template <REG T>
  typename std::enable_if<T == REG::IPOL, bool>::type
  setInputPolarity(uint8_t pinmask, PORT port, INPUT_POLARITY pol) {

    if (port_ != port) {
      ESP_LOGE(REG_TAG, "Pinmask does not belong to the expected port");
      return false;
    }
    if (pol == INPUT_POLARITY::INVERTED) {
      applyMask(pinmask);
    } else {
      clearMask(pinmask);
    }
    return true;
  }
  template <REG T>
  typename std::enable_if<T == REG::OLAT, bool>::type
  setOutputLatch(PIN pin, OUTPUT_LATCH lat) {

    if (port_ != getPortFromPin(pin)) {
      ESP_LOGE(REG_TAG, "Pin %d does not belong to the expected port",
               static_cast<int>(pin));
      return false;
    }
    uint8_t index = getIndexFromPin(pin);
    setBitField(static_cast<Field>(index), lat == OUTPUT_LATCH ::LOGIC_HIGH);

    return true;
  }
  template <REG T>
  typename std::enable_if<T == REG::OLAT, bool>::type
  setOutputLatch(uint8_t pinmask, PORT port, OUTPUT_LATCH lat) {

    if (port_ != port) {
      ESP_LOGE(REG_TAG, "Pinmask does not belong to the expected port");
      return false;
    }
    if (lat == OUTPUT_LATCH ::LOGIC_HIGH) {
      applyMask(pinmask);
    } else {
      clearMask(pinmask);
    }
    return true;
  }

  // INTERRUPT SPECIFIC
  template <REG T>
  typename std::enable_if<T == REG::GPINTEN, bool>::type
  setInterruptOnChange(PIN pin, INTR_ON_CHANGE_ENABLE en) {

    if (port_ != getPortFromPin(pin)) {
      ESP_LOGE(REG_TAG, "Pin %d does not belong to the expected port",
               static_cast<int>(pin));
      return false;
    }
    uint8_t index = getIndexFromPin(pin);
    setBitField(static_cast<Field>(index),
                en == INTR_ON_CHANGE_ENABLE::ENABLE_INTR_ON_CHANGE);

    return true;
  }
  template <REG T>
  typename std::enable_if<T == REG::GPINTEN, bool>::type
  setInterruptOnChange(uint8_t pinmask, PORT port, INTR_ON_CHANGE_ENABLE en) {

    if (port_ != port) {
      ESP_LOGE(REG_TAG, "Pinmask does not belong to the expected port");
      return false;
    }
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

    if (port_ != getPortFromPin(pin)) {
      ESP_LOGE(REG_TAG, "Pin %d does not belong to the expected port",
               static_cast<int>(pin));
      return false;
    }
    uint8_t index = getIndexFromPin(pin);
    setBitField(static_cast<Field>(index),
                cntrl == INTR_ON_CHANGE_CONTROL::COMPARE_WITH_DEFVAL);

    return true;
  }
  template <REG T>
  typename std::enable_if<T == REG::INTCON, bool>::type
  setInterruptType(uint8_t pinmask, PORT port, INTR_ON_CHANGE_CONTROL cntrl) {

    if (port_ != port) {
      ESP_LOGE(REG_TAG, "Pinmask does not belong to the expected port");
      return false;
    }
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

    if (port_ != getPortFromPin(pin)) {
      ESP_LOGE(REG_TAG, "Pin %d does not belong to the expected port",
               static_cast<int>(pin));
      return false;
    }
    uint8_t index = getIndexFromPin(pin);
    setBitField(static_cast<Field>(index),
                cmp == DEF_VAL_COMPARE::SAVE_LOGIC_HIGH);

    return true;
  }
  template <REG T>
  typename std::enable_if<T == REG::DEFVAL, bool>::type
  saveCompareValue(uint8_t pinmask, PORT port, DEF_VAL_COMPARE cmp) {

    if (port_ != port) {
      ESP_LOGE(REG_TAG, "Pinmask does not belong to the expected port");
      return false;
    }
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
  constexpr PORT getPortFromPin(PIN pin) const {
    return (static_cast<uint8_t>(pin) < 8) ? PORT::GPIOA : PORT::GPIOB;
  }
  constexpr uint8_t getIndexFromPin(PIN pin) const {
    uint8_t index = 0;
    PORT port = getPortFromPin(pin);
    uint8_t pinEnum = static_cast<uint8_t>(pin);
    return index = (port == PORT::GPIOB) ? (pinEnum - 8) : pinEnum;
  }
};

} // namespace MCP
#endif
