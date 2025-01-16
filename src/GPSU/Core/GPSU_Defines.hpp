#ifndef GPSU_DEFINES_HPP
#define GPSU_DEFINES_HPP
#include "stdint.h"
#include <driver/gpio.h>
namespace GPSU_CORE {

struct GPIO {
  struct DI {
    static constexpr gpio_num_t DI_PIN_0 = gpio_num_t::GPIO_NUM_2;
    static constexpr gpio_num_t DI_PIN_1 = gpio_num_t::GPIO_NUM_15;
    static constexpr gpio_num_t DI_PIN_2 = gpio_num_t::GPIO_NUM_13;
    static constexpr gpio_num_t DI_PIN_3 = gpio_num_t::GPIO_NUM_12;
  };

  struct DO {
    static constexpr gpio_num_t DO_PIN_0 = gpio_num_t::GPIO_NUM_32;
    static constexpr gpio_num_t DO_PIN_1 = gpio_num_t::GPIO_NUM_33;
    static constexpr gpio_num_t DO_PIN_2 = gpio_num_t::GPIO_NUM_25;
    static constexpr gpio_num_t DO_PIN_3 = gpio_num_t::GPIO_NUM_26;
  };
  struct I2C {
    static constexpr gpio_num_t I2C_SDA = gpio_num_t::GPIO_NUM_21;
    static constexpr gpio_num_t I2C_SCL = gpio_num_t::GPIO_NUM_22;
  };
};

enum class Process {
  TRAFFIC_LIGHT = 0,
  WATER_LEVEL = 1,
  STEPPER_MOTOR_CONTROL = 2,
  STATE_MACHINE = 3,
  OBJECT_COUNTER = 4,
  MOTOR_CONTROl = 5,
  ANY = -1,

};

} // namespace GPSU_CORE

#endif