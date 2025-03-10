#ifndef STATE_DEFINES_HPP
#define STATE_DEFINES_HPP
#include "stdint.h"
namespace TrafficLight {
enum class State { INIT = 0, RED_STATE, GREEN_STATE, YELLOW_STATE };
enum class CommandType {
  NONE,
  RESET,
  TURN_ON_RED,
  TURN_ON_YELLOW,
  TURN_ON_GREEN,
  TURN_OFF_RED,
  TURN_OFF_YELLOW,
  TURN_OFF_GREEN,
};
struct CommandData {
  int timeout_ms = 0;
  bool immediate_transition = false;
};
struct Command {
  bool check_exit = false;
  bool check_entry = false;
  CommandType exit_command = CommandType::NONE;
  CommandType entry_command = CommandType::NONE;
  CommandData exit_data = CommandData{};
  CommandData entry_data = CommandData{};
};
struct Config {
  int redTimeout_ms = 5000;
  int greenTimeout_ms = 3000;
  int yellowTimeout_ms = 2000;
  bool allowImmediateTransition = false;
  uint16_t error_blink_interval = 500;
  uint8_t max_errors = 3;
};
struct Inputs {
  struct Timer {
    int delta_time_ms = 0;
  } external_timer;
  struct UserCommand {
    bool button_pressed = false;
  } user_command;
};
struct Data {
  uint32_t current_time_ms = 0;
};
} // namespace TrafficLight
namespace WaterLevel {

enum class State : uint8_t {
  EMPTY = 0,
  START_FILLING,
  FILLING,
  DRAINING,
  PARTIAL_FILLED,
  FULL,
  OVERFLOW,
};
enum class CommandType {
  NONE,
  LEVEL_UPDATE,
  START_FILL,
  FILL_TO_LEVEL,
  STOP,
  START_DRAIN,
  TRIGGER_ALARM,
  CLEAR_ALARM,
};
struct CommandData {
  int pump_speed = 0;
  float target_level = 0.0f;
  bool open_drain_valve = false;
  bool open_fill_valve = false;
  bool alarm = false;
};
struct Command {
  bool check_exit = false;
  bool check_entry = false;
  CommandType exit_command = CommandType::NONE;
  CommandType entry_command = CommandType::NONE;
  CommandData exit_data = CommandData{};
  CommandData entry_data = CommandData{};
};
struct Config {
  float alarm_level = 2100;
  float max_capacity = 2000;
  float partial_mark = 500;
  float draining_mark = 1800;
  int fill_rate = 100;
  int drain_rate = -50;
  float sensor_min_sensitivity = 10;
  float hysteresis = 15;
};

struct Inputs {
  struct Sensors {
    float raw_adc_value = 0;
    float measured_level = 0;
    bool drain_valve_state = false;
    bool fill_valve_state = false;
  } sensors;
  struct UserCommand {
    float new_target_level = 0;
    bool fill_request = false;
    bool drain_request = false;
    bool stop = false;
  } user_command;
};
struct Data {
  float current_level = 0;
  float current_target_level = 0;
};
} // namespace WaterLevel
namespace SM {

enum class StateWaterLevel : uint8_t {
  EMPTY = 0,
  START_FILLING,
  FILLING,
  DRAINING,
  PARTIAL_FILLED,
  FULL,
  OVERFLOW,

};

struct StepperConfig {
  const float degreePerStep = 1.9;
  const uint16_t step_per_mm = 120;
};
struct StepperData {
  uint16_t currentDirection = 1; //+1 or -1
  uint16_t currentStep;
};

enum class StateStepperMotor : uint8_t {
  IDLE = 0,
  RUN,
  ERROR

};

} // namespace SM

#endif