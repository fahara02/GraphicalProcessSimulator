#ifndef STATE_DEFINES_HPP
#define STATE_DEFINES_HPP
#include "stdint.h"
namespace TrafficLight {
enum class State : uint8_t {
  INIT = 0,
  RED_STATE,
  GREEN_STATE,
  YELLOW_STATE,
  SYSTEM_FAULT
};
enum class Event : uint8_t {
  OK = 0,
  TIMER_FAULT,
  LED_FAULT,
};
enum class CommandType : uint8_t {
  NONE = 0,
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
  CommandType exit_command;
  CommandType entry_command;
  CommandData exit_data;
  CommandData entry_data;
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
  SYSTEM_FAULT
};
enum class CommandType : uint8_t {
  NONE = 0,
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
  int32_t target_level = 0;
  bool open_drain_valve = false;
  bool open_fill_valve = false;
  bool alarm = false;
};
struct Command {
  bool check_exit = false;
  bool check_entry = false;
  CommandType exit_command;
  CommandType entry_command;
  CommandData exit_data;
  CommandData entry_data;
};
struct Config {
  uint16_t alarm_level = 2100;
  uint16_t max_capacity = 2000;
  uint16_t partial_mark = 500;
  uint16_t draining_mark = 1800;
  int fill_rate = 100;
  int drain_rate = -50;
  int32_t sensor_min_sensitivity = 10;
  int32_t hysteresis = 15;
};

struct Inputs {
  struct Sensors {
    int32_t raw_adc_value = 0;
    int32_t measured_level = 0;
    bool drain_valve_state = false;
    bool fill_valve_state = false;
  } sensors;
  struct UserCommand {
    int32_t new_target_level = 0;
    bool fill_request = false;
    bool drain_request = false;
    bool stop = false;
  } user_command;
};
enum class Event : uint8_t {
  OK = 0,
  SENSOR_FAULT,
  PUMP_FAULT,
  DRAIN_VALVE_FAULT,
  FILL_VALVE_FAULT,
};
struct Data {
  unsigned long filling_start_time = 0;
  int32_t current_level = 0;
  int32_t current_target_level = 0;
};
} // namespace WaterLevel
namespace StepperMotor {
enum class State : uint8_t { IDLE = 0, MOVING_CW, MOVING_CCW, HOMING, ERROR };

enum class CommandType : uint8_t {
  NONE = 0,
  START_MOVE,
  STOP,
  SET_TARGET,
  HOME,
  CLEAR_ERROR,
  EMERGENCY_STOP
};

struct CommandData {
  int32_t target_position = 0;
  uint16_t speed_rpm = 60;
  bool direction = true; // CW=true, CCW=false
};
struct Command {
  bool check_exit = false;
  bool check_entry = false;
  CommandType exit_command;
  CommandType entry_command;
  CommandData exit_data;
  CommandData entry_data;
};
struct Config {
  // Physical properties
  float degrees_per_step = 1.8;
  uint16_t steps_per_revolution = 200;
  uint16_t max_steps = 2000; // Mechanical limit

  // Electrical properties
  uint16_t max_rpm = 300;
  uint16_t hold_current = 30;  // Percentage
  uint16_t move_current = 100; // Percentage

  // Safety
  uint32_t stall_timeout = 2000;  // ms
  int32_t position_tolerance = 5; // Steps
};

struct Inputs {
  struct Sensors {
    // PCNT inputs
    int16_t pulse_count = 0;
    bool direction_state = true;

    // Limit switches
    bool home_switch = false;
    bool cw_limit = false;
    bool ccw_limit = false;

    // Driver feedback
    bool driver_fault = false;
    bool over_temp = false;
  } sensors;

  struct UserCommand {
    int32_t target_position = 0;
    bool start_move = false;
    bool stop = false;
    bool home_request = false;
  } user_command;
};

enum class Event : uint8_t {
  OK = 0,
  MOVEMENT_COMPLETE,
  HOMING_COMPLETE,
  STALL_DETECTED,
  OVERCURRENT,
  OVERTEMP,
  LIMIT_SW_ACTIVATED
};

struct Data {
  // Position tracking
  int32_t current_position = 0;
  int32_t target_position = 0;

  // Movement parameters
  uint16_t current_rpm = 0;
  bool current_direction = true;

  // Timing
  unsigned long move_start_time = 0;

  // Error tracking
  uint8_t error_code = 0;
};

// Pulse generation control structure
struct PulseControl {
  uint8_t step_pin;
  uint8_t dir_pin;
  uint16_t pulse_delay; // µs between pulses
  bool pulse_state;
};
} // namespace StepperMotor
#endif