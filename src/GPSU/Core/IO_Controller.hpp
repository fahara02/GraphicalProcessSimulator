#ifndef IO_CONTROLLER_HPP
#define IO_CONTROLLER_HPP

#include "GPSU_Defines.hpp"
#include "Poller.hpp"
#include <ADS1115_WE.h>
#include <Wire.h>
#include <driver/gpio.h>
#include <memory>
#include <unordered_set>
#include <vector>

namespace GPSU_CORE {

// Struct to hold GPIO configuration details
struct GPIO_Config {
  gpio_num_t pin;
  gpio_mode_t mode;
  gpio_pullup_t pull_up;
  gpio_pulldown_t pull_down;
  gpio_int_type_t intr_type;
};

class IO_Controller : public Poller<IO_Controller> {
private:
  std::unique_ptr<ADS1115_WE> ADC_MANAGER;
  Process selected_process;
  uint16_t _pollrate = 100;
  uint16_t _address = 0x48;
  bool _initialised_adc = false;
  mutable unsigned long _lastInitialised;
  std::vector<GPIO_Config> gpio_configs;
  std::unordered_set<gpio_num_t> allowed_output_pins;
  std::array<int, MAX_INPUTS> INPUT_ARRAY{};
  std::array<float, MAX_AI_CHANNEL> ANALOG_DATA{};

  explicit IO_Controller(Process sp = Process::ANY,
                         uint32_t pollrate = POLLER_PERIOD,
                         uint16_t I2C_ADDRESS = DEFAULT_I2C_ADDRESS)
      : Poller(this, POLLER_PERIOD),
        ADC_MANAGER(std::make_unique<ADS1115_WE>(I2C_ADDRESS)),
        selected_process(sp), _pollrate(pollrate), _address(I2C_ADDRESS) {

    setupGPIOS();
    init_GPIOS();
    resetPoll();
    configASSERT(setPoll());
  }
  bool setPoll() { return setPollrate(_pollrate); }
  void setupGPIOS() {
    gpio_configs.clear();
    allowed_output_pins.clear();

    switch (selected_process) {
    case Process::TRAFFIC_LIGHT:
      gpio_configs = {
          {GPIO::DI::DI_PIN_0, GPIO_MODE_INPUT, GPIO_PULLUP_DISABLE,
           GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE},
          {GPIO::DI::DI_PIN_1, GPIO_MODE_INPUT, GPIO_PULLUP_DISABLE,
           GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE},
          {GPIO::DI::DI_PIN_2, GPIO_MODE_INPUT, GPIO_PULLUP_DISABLE,
           GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE},
          {GPIO::DO::DO_PIN_0, GPIO_MODE_OUTPUT, GPIO_PULLUP_DISABLE,
           GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE},
          {GPIO::DO::DO_PIN_1, GPIO_MODE_OUTPUT, GPIO_PULLUP_DISABLE,
           GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE},
          {GPIO::DO::DO_PIN_2, GPIO_MODE_OUTPUT, GPIO_PULLUP_DISABLE,
           GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE}};
      allowed_output_pins = {GPIO::DO::DO_PIN_0, GPIO::DO::DO_PIN_1,
                             GPIO::DO::DO_PIN_2};
      break;

    case Process::WATER_LEVEL:
      gpio_configs = {
          {GPIO::DI::DI_PIN_0, GPIO_MODE_INPUT, GPIO_PULLUP_DISABLE,
           GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE},
          {GPIO::I2C::I2C_SDA, GPIO_MODE_OUTPUT_OD, GPIO_PULLUP_ENABLE,
           GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE},
          {GPIO::I2C::I2C_SCL, GPIO_MODE_OUTPUT_OD, GPIO_PULLUP_ENABLE,
           GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE},
          {GPIO::DO::DO_PIN_0, GPIO_MODE_OUTPUT, GPIO_PULLUP_DISABLE,
           GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE},
          {GPIO::DO::DO_PIN_1, GPIO_MODE_OUTPUT, GPIO_PULLUP_DISABLE,
           GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE}};
      allowed_output_pins = {GPIO::DO::DO_PIN_0, GPIO::DO::DO_PIN_1};
      break;

    case Process::STEPPER_MOTOR_CONTROL:
      gpio_configs = {
          {GPIO::DI::DI_PIN_0, GPIO_MODE_INPUT, GPIO_PULLUP_DISABLE,
           GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE},
          {GPIO::DI::DI_PIN_1, GPIO_MODE_INPUT, GPIO_PULLUP_DISABLE,
           GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE},
          {GPIO::DI::DI_PIN_2, GPIO_MODE_INPUT, GPIO_PULLUP_DISABLE,
           GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE},
          {GPIO::DI::DI_PIN_3, GPIO_MODE_INPUT, GPIO_PULLUP_DISABLE,
           GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE},
          {GPIO::DO::DO_PIN_0, GPIO_MODE_OUTPUT, GPIO_PULLUP_DISABLE,
           GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE},
          {GPIO::DO::DO_PIN_1, GPIO_MODE_OUTPUT, GPIO_PULLUP_DISABLE,
           GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE},
          {GPIO::DO::DO_PIN_2, GPIO_MODE_OUTPUT, GPIO_PULLUP_DISABLE,
           GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE},
          {GPIO::DO::DO_PIN_3, GPIO_MODE_OUTPUT, GPIO_PULLUP_DISABLE,
           GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE}};
      allowed_output_pins = {GPIO::DO::DO_PIN_0, GPIO::DO::DO_PIN_1,
                             GPIO::DO::DO_PIN_2, GPIO::DO::DO_PIN_3};
      break;

    case Process::STATE_MACHINE:
      gpio_configs = {
          {GPIO::DI::DI_PIN_0, GPIO_MODE_INPUT, GPIO_PULLUP_DISABLE,
           GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE},
          {GPIO::DI::DI_PIN_1, GPIO_MODE_INPUT, GPIO_PULLUP_DISABLE,
           GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE},
          {GPIO::DI::DI_PIN_2, GPIO_MODE_INPUT, GPIO_PULLUP_DISABLE,
           GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE},
          {GPIO::DI::DI_PIN_3, GPIO_MODE_INPUT, GPIO_PULLUP_DISABLE,
           GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE},
          {GPIO::DO::DO_PIN_0, GPIO_MODE_OUTPUT, GPIO_PULLUP_DISABLE,
           GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE},
          {GPIO::DO::DO_PIN_1, GPIO_MODE_OUTPUT, GPIO_PULLUP_DISABLE,
           GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE},
          {GPIO::DO::DO_PIN_2, GPIO_MODE_OUTPUT, GPIO_PULLUP_DISABLE,
           GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE},
          {GPIO::DO::DO_PIN_3, GPIO_MODE_OUTPUT, GPIO_PULLUP_DISABLE,
           GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE}};
      allowed_output_pins = {GPIO::DO::DO_PIN_0, GPIO::DO::DO_PIN_1,
                             GPIO::DO::DO_PIN_2, GPIO::DO::DO_PIN_3};
      break;

    case Process::OBJECT_COUNTER:
    case Process::MOTOR_CONTROl:
      gpio_configs = {
          {GPIO::DI::DI_PIN_0, GPIO_MODE_INPUT, GPIO_PULLUP_DISABLE,
           GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE},
          {GPIO::DI::DI_PIN_1, GPIO_MODE_INPUT, GPIO_PULLUP_DISABLE,
           GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE},
          {GPIO::DO::DO_PIN_0, GPIO_MODE_OUTPUT, GPIO_PULLUP_DISABLE,
           GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE},
          {GPIO::DO::DO_PIN_1, GPIO_MODE_OUTPUT, GPIO_PULLUP_DISABLE,
           GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE}

      };
      allowed_output_pins = {GPIO::DO::DO_PIN_0, GPIO::DO::DO_PIN_1};

      break;

    default:
      // General default configuration
      gpio_configs = {{GPIO::DI::DI_PIN_0, GPIO_MODE_INPUT, GPIO_PULLUP_DISABLE,
                       GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE}};
      break;
    }
  }
  bool init_ADC() {
    bool init_result = false;
    if (_initialised_adc) {

      ADC_MANAGER->init();
      ADC_MANAGER->setVoltageRange_mV(ADS1115_RANGE_6144);
      ADC_MANAGER->setCompareChannels(ADS1115_COMP_1_3);
      ADC_MANAGER->setCompareChannels(ADS1115_COMP_2_3);
      ADC_MANAGER->setMeasureMode(ADS1115_CONTINUOUS);
      init_result = true;
    };
    return init_result;
  }
  void init_GPIOS() {
    for (const auto &config : gpio_configs) {
      gpio_config_t io_conf = {};
      io_conf.pin_bit_mask = 1ULL << config.pin;
      io_conf.mode = config.mode;
      io_conf.pull_down_en = config.pull_down;
      io_conf.pull_up_en = config.pull_up;
      io_conf.intr_type = config.intr_type;

      gpio_config(&io_conf);

      // Set default output state for OUTPUT pins
      if (config.mode == GPIO_MODE_OUTPUT) {
        gpio_set_level(config.pin, 0);
      }
    }
  }
  float readChannel(ADS1115_MUX channel) {
    float voltage = 0.0;
    ADC_MANAGER->setCompareChannels(channel);
    voltage = ADC_MANAGER->getResult_mV();
    return voltage;
  }
  void updateAnalogInputs() {
    if (!_initialised_adc) {
      return;
    }

    ANALOG_DATA[0] = readChannel(ADS1115_COMP_0_1); // Channel 0
    ANALOG_DATA[1] = readChannel(ADS1115_COMP_2_3); // Channel 1

    // Fill remaining slots with a default value (e.g., 0.0 or -1.0)
    for (size_t i = 2; i < ANALOG_DATA.size(); ++i) {
      ANALOG_DATA[i] = 0.0f;
    }
  }

  void updateInputs() {
    size_t index = 0;
    for (const auto &config : gpio_configs) {
      if (config.mode == GPIO_MODE_INPUT && index < MAX_INPUTS) {
        INPUT_ARRAY[index++] = gpio_get_level(config.pin);
      }
    }
    // Fill remaining slots with -1 to indicate unused slots
    for (; index < MAX_INPUTS; ++index) {
      INPUT_ARRAY[index] = -1;
    }
  }

public:
  static IO_Controller &
  getInstance(Process sp = Process::ANY, uint32_t pollrate = 100,
              uint16_t I2C_ADDRESS = DEFAULT_I2C_ADDRESS) {
    static IO_Controller instance(sp, pollrate, I2C_ADDRESS);
    return instance;
  }

  IO_Controller(const IO_Controller &) = delete;
  IO_Controller &operator=(const IO_Controller &) = delete;
  void resetPoll() const override { _lastInitialised = esp_timer_get_time(); };

  void setOutput(gpio_num_t pin, int level) {
    if (allowed_output_pins.find(pin) == allowed_output_pins.end()) {
      printf("Error: GPIO %d is not an allowed output pin for this process.\n",
             pin);
      return;
    }
    gpio_set_level(pin, level);
  }
  void updateData() {
    updateInputs();
    updateAnalogInputs();
  }

  Process getSelectedProcess() const { return selected_process; }
  std::array<int, MAX_INPUTS> getInputs() const { return INPUT_ARRAY; }
};

} // namespace GPSU_CORE

#endif // IO_CONTROLLER_HPP
