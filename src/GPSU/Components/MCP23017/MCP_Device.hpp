#ifndef MCP_DEVICE_HPP
#define MCP_DEVICE_HPP
#include "MCP_Constants.hpp"
#include "MCP_defines.hpp"
namespace MCP {

struct register_icon_t {
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
  register_icon_t() : value(0) {}
  register_icon_t(uint8_t setting) : value(setting) {}

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
class MCPDeviceBase {
public:
  uint8_t address[MAX_REG_PER_DEVICE];
  virtual ~MCPDeviceBase() = default;
  using Field = MCP::register_icon_t::Field;

  virtual void updateRegisters() = 0;
  virtual uint8_t *generateAddress(PORT port) = 0;

  virtual bool configure(const uint8_t &settings) {
    bool status = settings_.configure(settings);
    if (status) {
      settings_ = register_icon_t(settings);
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
  register_icon_t settings_;
  MCP::MCP_MODEL model_;
};

template <typename RegEnum, MCP::MCP_MODEL model> //
class MCPDevice : public MCPDeviceBase {
private:
  RegEnum REG;
  void (*callback)() = nullptr;

public:
  using RegEnumType = RegEnum;
  MCPDevice() {
    setModel(model);
    updateRegisters();
  }
  MCPDevice(void (*cb)()) : callback(cb) {
    setModel(model);
    updateRegisters();
  }

  void updateRegisters() override {
    if (callback) {
      callback();
    }
  }
  uint8_t *generateAddress(PORT port) override {

    for (size_t i = 0; i < MAX_REG_PER_PORT; ++i) {
      address[i] = generateAddressImpl(static_cast<RegEnum>(i), port);
    }
    return address;
  }

protected:
  constexpr uint8_t generateAddressImpl(RegEnum reg, PORT port) const {
    uint8_t baseAddress = static_cast<uint8_t>(reg);

    if (getBankMode()) {
      // BANK = 1: Separate PORTA and PORTB
      return baseAddress + (port == PORT::GPIOB ? 0x10 : 0x00);
    } else {
      // BANK = 0: Paired A/B registers
      return baseAddress + (port == PORT::GPIOB ? 0x01 : 0x00);
    }
    // Redundant return to silence warnings
    return baseAddress;
  }
};

struct MCPFamily {
  MCP::MCP_MODEL model;
  MCP::MCP_MODEL getModel() { return model; }
};
struct MCP23017 : public MCPFamily {
  static constexpr MCP::MCP_MODEL model = MCP::MCP_MODEL ::MCP23017;
  using RegEnumType = MCP::MCP_23X17::REG;
  using PinEnumType = MCP::MCP_23X17::PIN;
  using PinType = MCP::Pin<PinEnumType>;
  using DeviceType = MCP::MCPDevice<RegEnumType, model>;
  DeviceType device;
};
struct MCP23S17 : public MCPFamily {
  static constexpr MCP::MCP_MODEL model = MCP::MCP_MODEL ::MCP23S17;
  using RegEnumType = MCP::MCP_23X17::REG;
  using PinEnumType = MCP::MCP_23X17::PIN;
  using PinType = MCP::Pin<PinEnumType>;
  using DeviceType = MCP::MCPDevice<RegEnumType, model>;
  DeviceType device;
};
struct MCP23018 : public MCPFamily {
  static constexpr MCP::MCP_MODEL model = MCP::MCP_MODEL ::MCP23018;
  using RegEnumType = MCP::MCP_23X18::REG;
  using PinEnumType = MCP::MCP_23X18::PIN;
  using PinType = MCP::Pin<PinEnumType>;
  using DeviceType = MCP::MCPDevice<RegEnumType, model>;
  DeviceType device;
};
struct MCP23S18 : public MCPFamily {
  static constexpr MCP::MCP_MODEL model = MCP::MCP_MODEL ::MCP23S18;
  using RegEnumType = MCP::MCP_23X18::REG;
  using PinEnumType = MCP::MCP_23X18::PIN;
  using PinType = MCP::Pin<PinEnumType>;
  using DeviceType = MCP::MCPDevice<RegEnumType, model>;
  DeviceType device;
};

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