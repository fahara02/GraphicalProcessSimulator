#ifndef STATE_DEFINES_HPP
#define STATE_DEFINES_HPP
#include "stdint.h"
namespace TrafficLight {
enum class Mode : uint8_t { AUTO, MANUAL };
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
  int timeout1_ms = 0;
  int timeout2_ms = 0;
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
  bool new_input = false;
  struct Timer {
    int external_delta_time_ms = 0;
    bool internal_timer1_expired = false;
    bool internal_timer2_expired = false;
    int current_time_ms = 0;
  } timer;
  struct UserCommand {
    bool turn_on_red = false;
    bool turn_on_yellow = false;
    bool turn_on_green = false;
    bool button_pressed = false;
  } user_command;
};
struct Data {
  uint32_t current_time_ms = 0;
  bool timer_expired = false;
};
} // namespace TrafficLight
namespace WaterLevel {
enum class Mode : uint8_t { AUTO, MANUAL };
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
  int timeout1_ms = 0;
  int timeout2_ms = 0;
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
  bool new_input = false;
  struct Timer {
    int external_delta_time_ms = 0;
    bool internal_timer1_expired = false;
    bool internal_timer2_expired = false;
    int current_time_ms = 0;
  } timer;
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
enum class Mode : uint8_t { AUTO, MANUAL };
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
  int timeout1_ms = 0;
  int timeout2_ms = 0;
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
  bool new_input = false;
  struct Timer {
    int external_delta_time_ms = 0;
    bool internal_timer1_expired = false;
    bool internal_timer2_expired = false;
    int current_time_ms = 0;
  } timer;
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
    bool clear_error = false;
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
namespace Objects {

enum class State : uint8_t {
  INIT = 0,
  PLACED,
  SENSED,
  AT_PICKER,
  PICKED,
  FAILED
};

struct Data {
  uint32_t placement_time_ms = 0;      // Time when object was placed (ms)
  uint32_t sensor_trigger_time_ms = 0; // Time when sensor should trigger (ms)
  uint32_t picker_arrival_time_ms = 0; // Time when object reaches picker (ms)
  uint32_t pick_attempt_time_ms = 0;
  uint32_t pick_deadline_time_ms = 0;
};

} // namespace Objects
namespace ObjectCounter {
enum class Mode : uint8_t { AUTO, MANUAL };
enum class State : uint8_t { INIT = 0, READY, RUNNING, FAULT, E_STOP, RESET };

struct Object {
  Objects::State state;
  Objects::Data data;
  bool triggered_sensor = false;
  bool reached_picker = false;
};
enum class Event : uint8_t {
  OK = 0,
  NONE,
  SENSOR_TRIGGERED,
  PICK_SUCCESS,
  PICK_TIMEOUT,
  E_STOP_ACTIVATED,
  SAFETY_TIMEOUT,
  SENSOR_RUNWAY
};
enum class CommandType : uint8_t {
  NONE = 0,
  RESET,
  START_CONVEYOR,
  STOP_CONVEYOR,
  TRIGGER_SENSOR,
  ACTIVATE_PICKER,
  SOUND_ALARM,
  CLEAR_ALARM,
  ENABLE_MANUAL_MODE

};
struct CommandData {
  int timeout1_ms = 0;
  int timeout2_ms = 0;
  uint32_t duration_ms = 0;
  uint8_t retry_count = 0;
  bool requires_acknowledgment = false;
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
  static constexpr uint16_t object_length_mm = 50;
  static constexpr uint16_t conveyer_velocity_mmps = 120;
  static constexpr uint16_t sensor_position_mm = 500;
  static constexpr uint16_t picker_position_mm = 800;
  static constexpr int32_t object_placement_rate_sec = 5;
  static constexpr uint32_t simulated_pick_delay_ms = 20;
  static constexpr uint8_t max_objects = 10;

  // Derived timings (calculated once during initialization)
  uint32_t sensor_trigger_delay_ms;      //
  uint32_t picker_arrival_delay_ms;      //
  uint32_t object_placement_interval_ms; //
  uint32_t auto_picking_time_limit_ms;   //
  uint32_t manual_picking_time_limit_ms; //

  Config() {
    sensor_trigger_delay_ms =
        (sensor_position_mm * 1000) / conveyer_velocity_mmps;
    picker_arrival_delay_ms =
        (picker_position_mm * 1000) / conveyer_velocity_mmps;
    object_placement_interval_ms = object_placement_rate_sec * 1000;
    auto_picking_time_limit_ms =
        picker_arrival_delay_ms + simulated_pick_delay_ms;
    manual_picking_time_limit_ms = 2000;
    ;
  }
};
struct Inputs {
  bool new_input = false;
  struct Timer {
    // placement timer, resets after placement
    bool internal_timer1_expired = false;
    // timer from placement to end ,resets after picked or fail
    bool internal_timer2_expired = false;
    uint32_t current_placement_time_ms = 0;
    uint32_t current_placed_object_ms = 0;
  } timer;
  struct Sensors {
    bool photoeye_active = false;       // Main object detection sensor
    bool pick_position_sensor = false;  // Picker mechanism position
    bool emergency_stop_active = false; // E-stop button state
    bool safety_guard_closed = true;    // Machine safety guard
  } sensors;
  struct UserCommand {

    bool start = false;
    bool stop = false;
    bool reset = false;
    bool acknowledge_sensor = false;
    bool pick_arrived = false;
    bool pick_now = false;    // Manual pick trigger
    bool mode_switch = false; // Auto/Manual toggle
  } user_command;
};
struct Data {
  bool picker_busy = false;
  struct Timing {
    uint32_t current_time_ms = 0;
    uint32_t last_placement_time_ms = 0;
    uint32_t system_uptime = 0;
    uint32_t conveyor_runtime = 0;
  } timing;

  struct ProductionData {
    uint16_t total_objects_detected = 0;
    uint16_t successful_picks = 0;
    uint16_t failed_picks = 0;
    uint16_t conveyor_starts = 0;
    float average_cycle_time = 0.0f;
  } production;
};
} // namespace ObjectCounter
#endif