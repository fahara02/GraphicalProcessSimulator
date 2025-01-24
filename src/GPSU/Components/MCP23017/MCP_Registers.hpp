#ifndef MCP_REGISTERS_HPP
#define MCP_REGISTERS_HPP
#include "MCP_Constants.hpp"
#include "MCP_Primitives.hpp"
#include <functional>
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
public:
  uint8_t address[MAX_REG_PER_DEVICE];
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

template <typename RegEnum, MCP::MCP_MODEL model> //
class ControlRegister : public ioconBase {
private:
  RegEnum REG;
  std::function<void()> callback;

public:
  using RegEnumType = RegEnum;
  ControlRegister() { setModel(model); }

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
  MCP::MCP_MODEL model;
  uint8_t enumIndex;
  MCP::REG_FUNCTION function;
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
  virtual void setFunction(MCP::REG_FUNCTION fn) { function = fn; };

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
      uint8_t bit7 : 1;     //!< Controls how the registers are addressed
      uint8_t bit6 : 1;     //!< INT Pins Mirror bit
      uint8_t bit5 : 1;     //!< Sequential Operation mode bit
      uint8_t bit4 : 1;     //!< Slew Rate control bit for SDA output
      uint8_t bit3 : 1;     //!< Enables hardware addressing
      uint8_t bit2 : 1;     //!< Configures the INT pin as an open-drain output
      uint8_t bit1 : 1;     //!< Sets the polarity of the INT output pin
      uint8_t reserved : 1; //!< Reserved bit (unused)
    };
  };
};

template <typename RegEnum, MCP::MCP_MODEL model> //
class MCPRegister : public RegisterBase {
private:
  RegEnum reg;
  PORT port;
  bool bankMode;
  uint8_t regAddress;

public:
  MCPRegister(RegEnum r, PORT p, bool bm)
      : reg(r), port(p), bankMode(bm), regAddress(calculateAddress(r, p)) {

    if (static_cast<uint8_t>(r) == 0) {
      setValue(1);
    } else {
      setValue(0);
    }
  }
  void setenumIndex(uint8_t index) override {
    enumIndex = static_cast<uint8_t>(reg);
    updateRegisterAddress();
  }
  void setModel(MCP::MCP_MODEL m) override { model = m; }
  void updateRegisterAddress() override {
    setAddress(calculateAddress(reg, port));
  }
  void setAddress(uint8_t address) override { regAddress = address; }

  uint8_t getAddress() const override { return regAddress; }

private:
  constexpr uint8_t calculateAddress(RegEnum reg, PORT port) const {
    uint8_t baseAddress = static_cast<uint8_t>(reg);

    if (bankMode) {
      // BANK = 1: Separate PORTA and PORTB
      return baseAddress + (port == PORT::GPIOB ? 0x10 : 0x00);
    } else {
      // BANK = 0: Paired A/B registers
      return (baseAddress * 2) + (port == PORT::GPIOB ? 0x01 : 0x00);
    }
    // Redundant return to silence warnings
    return baseAddress;
  }
};

namespace MCP_Chip {

struct MCPFamily {
  MCP::MCP_MODEL model;
  MCP::MCP_MODEL getModel() { return model; }
};
struct MCP23017 : public MCPFamily {
  static constexpr MCP::MCP_MODEL model = MCP::MCP_MODEL ::MCP23017;
  using RegEnumType = MCP::MCP_23X17::REG;
  using PinEnumType = MCP::MCP_23X17::PIN;
  using PinType = MCP::Pin<PinEnumType>;
  using ICONType = MCP::ControlRegister<RegEnumType, model>;
  using RegisterType = MCP::MCPRegister<RegEnumType, model>;
  ICONType iocon;
};
struct MCP23S17 : public MCPFamily {
  static constexpr MCP::MCP_MODEL model = MCP::MCP_MODEL ::MCP23S17;
  using RegEnumType = MCP::MCP_23X17::REG;
  using PinEnumType = MCP::MCP_23X17::PIN;
  using PinType = MCP::Pin<PinEnumType>;
  using ICONType = MCP::ControlRegister<RegEnumType, model>;
  using RegisterType = MCP::MCPRegister<RegEnumType, model>;
  ICONType iocon;
};
struct MCP23018 : public MCPFamily {
  static constexpr MCP::MCP_MODEL model = MCP::MCP_MODEL ::MCP23018;
  using RegEnumType = MCP::MCP_23X18::REG;
  using PinEnumType = MCP::MCP_23X18::PIN;
  using PinType = MCP::Pin<PinEnumType>;
  using ICONType = MCP::ControlRegister<RegEnumType, model>;
  using RegisterType = MCP::MCPRegister<RegEnumType, model>;
  ICONType iocon;
};
struct MCP23S18 : public MCPFamily {
  static constexpr MCP::MCP_MODEL model = MCP::MCP_MODEL ::MCP23S18;
  using RegEnumType = MCP::MCP_23X18::REG;
  using PinEnumType = MCP::MCP_23X18::PIN;
  using PinType = MCP::Pin<PinEnumType>;
  using ICONType = MCP::ControlRegister<RegEnumType, model>;
  using RegisterType = MCP::MCPRegister<RegEnumType, model>;
  ICONType iocon;
};
}; // namespace MCP_Chip

using MCP23017Pin = MCP_Chip::MCP23017::PinType;
constexpr MCP23017Pin GPB0(MCP_23X17::PIN::PIN8);

} // namespace MCP
#endif

// Configuration methods that use `register_icon_ t`

//   void printRegisters() const {
//     printf("=== PORTA Registers ===\n");
//     for (size_t i = 0; i < MAX_REG_PER_PORT; ++i) {
//       printf("Index: %zu, Register: %d, Address: 0x%02X\n", i,
//              static_cast<int>(registersPortA[i].reg),
//              registersPortA[i].address);
//     }

//     printf("\n=== PORTB Registers ===\n");
//     for (size_t i = 0; i < MAX_REG_PER_PORT; ++i) {
//       printf("Index: %zu, Register: %d, Address: 0x%02X\n", i,
//              static_cast<int>(registersPortB[i].reg),
//              registersPortB[i].address);
//     }
//   }