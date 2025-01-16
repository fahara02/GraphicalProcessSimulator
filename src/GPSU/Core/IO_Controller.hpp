#ifndef IO_CONTROLLER_HPP
#define IO_CONTROLLER_HPP

#include "GPSU_Defines.hpp"
#include <driver/gpio.h>
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

class IO_Controller {
private:
  Process selected_process;
  std::vector<GPIO_Config> gpio_configs;
  std::unordered_set<gpio_num_t> allowed_output_pins;

  explicit IO_Controller(Process sp = Process::ANY) : selected_process(sp) {
    setupGPIOS();
    init_GPIOS();
  }

  void setupGPIOS() {
    gpio_configs.clear();

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

public:
  static IO_Controller &getInstance(Process sp = Process::ANY) {
    static IO_Controller instance(sp);
    return instance;
  }

  IO_Controller(const IO_Controller &) = delete;
  IO_Controller &operator=(const IO_Controller &) = delete;

  void updateGPIO(gpio_num_t pin, int level) {
    if (allowed_output_pins.find(pin) == allowed_output_pins.end()) {
      printf("Error: GPIO %d is not an allowed output pin for this process.\n",
             pin);
      return;
    }
    gpio_set_level(pin, level);
  }

  Process getSelectedProcess() const { return selected_process; }
};

} // namespace GPSU_CORE

#endif // IO_CONTROLLER_HPP
