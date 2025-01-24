#ifndef MCP_DEVICE_HPP
#define MCP_DEVICE_HPP
#include "MCP_Constants.hpp"
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
  bool getBankMode() { return bank; }
  uint8_t getSettings() const { return static_cast<uint8_t>(value); }
};

//
class MCPDeviceBase {
public:
  virtual ~MCPDeviceBase() = default;
  using Field = MCP::register_icon_t::Field;

  virtual void configure(const MCP::register_icon_t &settings) = 0;
  virtual void updateRegisters() = 0;
  virtual void setModel(MCP::MCP_MODEL model) { model_ = model; }

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

  virtual uint8_t getSettings() const { return settings_.getSettings(); }

protected:
  register_icon_t settings_;
  MCP::MCP_MODEL model_;
};

template <typename RegEnum, MCP::MCP_MODEL model> //
class MCPDevice : public MCPDeviceBase {
private:
  RegEnum REG;
  struct regMap {
    RegEnum reg;
    uint8_t address;
  };
  std::array<regMap, MAX_REG_PER_PORT> registersPortA{};
  std::array<regMap, MAX_REG_PER_PORT> registersPortB{};

public:
  using RegEnumType = RegEnum;
  MCPDevice() {
    setModel(model);
    updateRegisters();
  }
  void configure(const MCP::register_icon_t &settings) override {
    settings_ = settings;
  }
  void updateRegisters() override {
    for (size_t i = 0; i < MAX_REG_PER_PORT; ++i) {
      RegEnum reg = static_cast<RegEnum>(i);
      registersPortA[i] = {reg, generateAddress(reg, PORT::GPIOA)};
      registersPortB[i] = {reg, generateAddress(reg, PORT::GPIOB)};
    }
  }

  uint8_t getAddress(RegEnum reg, PORT port) const {

    const auto &registers =
        (port == PORT::GPIOA) ? registersPortA : registersPortB;

    for (size_t i = 0; i < registers.size(); ++i) {
      if (registers[i].reg == reg) {

        return registers[i].address;
      }
    }
    return 0xFF;
  }

  // Configuration methods that use `register_icon_ t`

  void printRegisters() const {
    printf("=== PORTA Registers ===\n");
    for (size_t i = 0; i < MAX_REG_PER_PORT; ++i) {
      printf("Index: %zu, Register: %d, Address: 0x%02X\n", i,
             static_cast<int>(registersPortA[i].reg),
             registersPortA[i].address);
    }

    printf("\n=== PORTB Registers ===\n");
    for (size_t i = 0; i < MAX_REG_PER_PORT; ++i) {
      printf("Index: %zu, Register: %d, Address: 0x%02X\n", i,
             static_cast<int>(registersPortB[i].reg),
             registersPortB[i].address);
    }
  }

private:
  constexpr uint8_t generateAddress(RegEnum reg, PORT port) {
    uint8_t baseAddress = static_cast<uint8_t>(reg);

    if (settings_.getBankMode()) {
      // BANK = 1: Separate PORTA and PORTB
      return baseAddress + (port == PORT::GPIOB ? 0x10 : 0x00);
    } else {
      // BANK = 0: Paired A/B registers
      return baseAddress + (port == PORT::GPIOB ? 0x01 : 0x00);
    }
  }
};

} // namespace MCP
#endif