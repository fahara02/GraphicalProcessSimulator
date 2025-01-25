#ifndef MCP_REGISTERS_HPP
#define MCP_REGISTERS_HPP
#include "MCP_Constants.hpp"
#include "MCP_Primitives.hpp"
#include <functional>
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
  MCP::MCP_MODEL model_;
  uint8_t enumIndex;
  bool readOnly = false;

public:
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
  virtual void setenumIndex(uint8_t index) = 0;
  virtual void setAddress(uint8_t address) = 0;
  virtual uint8_t getAddress() const = 0;

  virtual void setReadonly() { readOnly = true; }

  virtual void applyMask(uint8_t mask) {
    { value |= mask; }
  }

  virtual void clearMask(uint8_t mask) { value &= ~mask; }

  virtual bool setValue(uint8_t newValue) {

    value = newValue;

    return true;
  }

  virtual void setBitField(Field field, bool value) {
    uint8_t *fields = reinterpret_cast<uint8_t *>(this);
    if (value)
      *fields |= (1 << field);
    else
      *fields &= ~(1 << field);
  }

  virtual bool getBitField(Field field) const {
    const uint8_t *fields = reinterpret_cast<const uint8_t *>(this);
    return (*fields & (1 << field)) != 0;
  }

  virtual uint8_t getValue() const { return value; }

private:
  union {
    uint8_t value; // Full register value
    struct {
      uint8_t bit7 : 1; //!< Controls how the registers are addressed
      uint8_t bit6 : 1; //!< INT Pins Mirror bit
      uint8_t bit5 : 1; //!< Sequential Operation mode bit
      uint8_t bit4 : 1; //!< Slew Rate control bit for SDA output
      uint8_t bit3 : 1; //!< Enables hardware addressing
      uint8_t bit2 : 1; //!< Configures the INT pin as an open-drain output
      uint8_t bit1 : 1; //!< Sets the polarity of the INT output pin
      uint8_t bit0 : 1; //!< Reserved bit (unused)
    };
  };
};

//

class MCPRegister : public RegisterBase {
private:
  REG reg_;
  PORT port_;
  bool bankMode_;
  uint8_t regAddress_;

public:
  MCPRegister(MCP::MCP_MODEL m, REG r, PORT p, bool bm)
      : reg_(r), port_(p), bankMode_(bm), regAddress_(calculateAddress(r, p)) {

    if (static_cast<uint8_t>(r) == 0) {
      setValue(1);
    } else {
      setValue(0);
    }

    setModel(m);
  }

  void setenumIndex(uint8_t index) override {
    enumIndex = static_cast<uint8_t>(reg_);
    updateRegisterAddress();
  }
  void setModel(MCP::MCP_MODEL m) override { model_ = m; }
  void updateRegisterAddress() override {
    setAddress(calculateAddress(reg_, port_));
  }
  void setAddress(uint8_t address) override { regAddress_ = address; }

  uint8_t getAddress() const override { return regAddress_; }

  template <REG T>
  typename std::enable_if<T == MCP::REG::IODIR, bool>::type
  setPinMode(MCP::PIN pin, MCP::GPIO_MODE mode) {

    if (port_ != getPortFromPin(pin)) {
      ESP_LOGE(REG_TAG, "Pin %d does not belong to the expected port",
               static_cast<int>(pin));
      return false;
    }
    uint8_t index = getIndexFromPin(pin);
    setBitField(static_cast<Field>(index), mode == MCP::GPIO_MODE::GPIO_INPUT);

    return true;
  }
  template <REG T>
  typename std::enable_if<T == MCP::REG::IODIR, bool>::type
  setPinMode(uint8_t pinmask, MCP::PORT port, MCP::GPIO_MODE mode) {

    if (port_ != port) {
      ESP_LOGE(REG_TAG, "Pinmask does not belong to the expected port");
      return false;
    }

    if (mode == MCP::GPIO_MODE::GPIO_INPUT) {
      applyMask(pinmask);
    } else {
      clearMask(pinmask);
    }

    return true;
  }

  template <REG T>
  typename std::enable_if<T == MCP::REG::GPPU, bool>::type
  setPullType(PIN pin, MCP::PULL_MODE mode) {

    if (port_ != getPortFromPin(pin)) {
      ESP_LOGE(REG_TAG, "Pin %d does not belong to the expected port",
               static_cast<int>(pin));
      return false;
    }
    uint8_t index = getIndexFromPin(pin);
    setBitField(static_cast<Field>(index),
                mode == MCP::PULL_MODE::ENABLE_PULLUP);

    return true;
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
  constexpr MCP::PORT getPortFromPin(MCP::PIN pin) const {
    return (static_cast<uint8_t>(pin) < 8) ? MCP::PORT::GPIOA
                                           : MCP::PORT::GPIOB;
  }
  constexpr uint8_t getIndexFromPin(MCP::PIN pin) const {
    uint8_t index = 0;
    MCP::PORT port = getPortFromPin(pin);
    uint8_t pinEnum = static_cast<uint8_t>(pin);
    return index = (port == MCP::PORT::GPIOB) ? (pinEnum - 8) : pinEnum;
  }
};

} // namespace MCP
#endif
