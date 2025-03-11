// WaterLevelSM.hpp
#pragma once
#include "StateMachines/StateDefines.hpp"
#include "StateMachines/StateMachine.hpp"
#include "Utility/gpsuUtility.hpp"

namespace WaterLevel {

struct Context {
  using Config = WaterLevel::Config;
  using Data = WaterLevel::Data;
  using Inputs = WaterLevel::Inputs;
  using Event = WaterLevel::Event;
  using Mode = TrafficLight::Mode;
  State previous_state;
  Config config;
  Data data;
  Inputs inputs;
  Event event;
  Mode mode = Mode::AUTO;
};

struct Traits {
  using Context = WaterLevel::Context;
  using Config = WaterLevel::Config;
  using Data = WaterLevel::Data;
  using Inputs = WaterLevel::Inputs;
  using Event = WaterLevel::Event;
  using State = WaterLevel::State;
  using Command = WaterLevel::Command;
  using Guard = bool (*)(const Context &);
  using ExitAction = Command (*)(const Context &);
  using EntryAction = Command (*)(const Context &);

  static constexpr bool has_exit_actions = true;
  static constexpr bool has_entry_actions = true;
  static constexpr uint16_t state_count = 8;
  static constexpr uint16_t transition_count = 18;

  struct Transition {
    State from;
    State to;
    Guard condition;
    EntryAction entry_action;
    ExitAction exit_action;
  };
  static bool systemFault(const Context &ctx) {
    return ctx.event == Event::DRAIN_VALVE_FAULT ||
           ctx.event == Event::FILL_VALVE_FAULT ||
           ctx.event == Event::PUMP_FAULT || ctx.event == Event::SENSOR_FAULT;
  }
  static bool systemFaultCleared(const Context &ctx) {
    return ctx.event == Event::OK;
  }
  static inline bool detectedOverflow(const Context &ctx) {
    return ctx.data.current_level > ctx.config.max_capacity;
  }
  static inline bool overflowResolved(const Context &ctx) {
    return ctx.data.current_level <= ctx.config.max_capacity;
  }

  static inline bool waterDetected(const Context &ctx) {
    // water is detected and filling should start.
    return ctx.inputs.sensors.raw_adc_value > ctx.config.sensor_min_sensitivity;
  }
  static inline bool startFillingCondition(const Context &ctx) {
    return ctx.inputs.user_command.fill_request;
  }
  static inline bool fillingTimeout(const Context &ctx) {
    static unsigned long startTime = millis();
    if (ctx.previous_state != State::START_FILLING)
      startTime = millis();
    return (millis() - ctx.data.filling_start_time > 5000); // 5-second timeout
  }

  static inline bool reachedEmpty(const Context &ctx) {
    return ctx.inputs.sensors.measured_level <= 0.0f + ctx.config.hysteresis;
  }
  static inline bool reachedPartial(const Context &ctx) {
    // Transition when the water level is moderately high (e.g., > 500 units)
    return ctx.data.current_level >= ctx.config.partial_mark;
  }
  static inline bool reachedLevel(const Context &ctx) {
    // Transition when the water level reached a particular value
    return ctx.data.current_level + ctx.config.hysteresis >=
           ctx.data.current_target_level;
  }
  static inline bool reachedFull(const Context &ctx) {
    return ctx.data.current_level >=
           (ctx.config.max_capacity - ctx.config.hysteresis);
  }
  static inline bool droppedBelowFull(const Context &ctx) {
    return ctx.data.current_level <
           (ctx.config.draining_mark - ctx.config.hysteresis);
  }
  static inline bool startDrainingCondition(const Context &ctx) {
    return ctx.inputs.user_command.drain_request;
  }
  static inline bool stopCondition(const Context &ctx) {
    return ctx.inputs.user_command.stop;
  }
  struct entryActions {

    static Command handleOverflow(const Context &ctx) {
      // Stop filling or trigger an alarm in case of overflow.
      Command cmd;
      cmd.check_entry = true;
      cmd.check_exit = false;
      cmd.entry_command = CommandType::TRIGGER_ALARM;
      cmd.entry_data.pump_speed = 0;
      cmd.entry_data.open_fill_valve = false;
      cmd.entry_data.alarm = true;
      return cmd;
    }

    static Command emptyToFilling(const Context &ctx) {
      Command cmd;
      cmd.check_entry = true;
      cmd.check_exit = false;
      cmd.entry_command = CommandType::START_FILL;
      cmd.entry_data.pump_speed = ctx.config.fill_rate; // Start filling
      cmd.entry_data.open_fill_valve = true;
      return cmd;
    }
    static Command startDraining(const Context &ctx) {
      Command cmd;
      cmd.check_entry = true;
      cmd.check_exit = false;
      cmd.entry_command = CommandType::START_DRAIN;
      cmd.entry_data.pump_speed = ctx.config.drain_rate;
      cmd.entry_data.open_drain_valve = true;
      return cmd;
    }

    static Command stop(const Context &ctx) {
      Command cmd;
      cmd.check_entry = true;
      cmd.check_exit = false;
      cmd.entry_command = CommandType::STOP;
      cmd.entry_data.pump_speed = 0;
      cmd.entry_data.open_fill_valve = false;
      cmd.entry_data.open_drain_valve = false;
      return cmd;
    }
    static Command fullToDraining(const Context &ctx) {
      Command cmd;
      cmd.check_entry = true;
      cmd.check_exit = false;
      cmd.entry_command = CommandType::START_DRAIN;
      cmd.entry_data.pump_speed = ctx.config.drain_rate;
      cmd.entry_data.open_drain_valve = true;
      return cmd;
    }

    static Command partialToFilling(const Context &ctx) {
      Command cmd;
      cmd.check_entry = true;
      cmd.check_exit = false;
      cmd.entry_command = CommandType::START_FILL;
      cmd.entry_data.pump_speed = ctx.config.fill_rate;
      cmd.entry_data.target_level = ctx.inputs.user_command.new_target_level;
      cmd.entry_data.open_fill_valve = true;
      return cmd;
    }

    static Command partialToDraining(const Context &ctx) {
      Command cmd;
      cmd.check_entry = true;
      cmd.check_exit = false;
      cmd.entry_command = CommandType::START_DRAIN;
      cmd.entry_data.pump_speed = ctx.config.drain_rate;
      cmd.entry_data.target_level = ctx.inputs.user_command.new_target_level;
      cmd.entry_data.open_drain_valve = true;
      return cmd;
    }
  };
  struct exitActions {
    static Command clearAlarm(const Context &ctx) {
      Command cmd;
      cmd.check_entry = false;
      cmd.check_exit = true;
      cmd.exit_command = CommandType::CLEAR_ALARM;
      cmd.exit_data.alarm = false; // Turn off the alarm
      return cmd;
    }
  };

  static constexpr std::array<Transition, transition_count> transitions = {{
      // Transitions to OVERFLOW (Entry: Trigger Alarm)
      {State::EMPTY, State::OVERFLOW, detectedOverflow,
       entryActions::handleOverflow, nullptr},
      {State::FILLING, State::OVERFLOW, detectedOverflow,
       entryActions::handleOverflow, nullptr},
      {State::PARTIAL_FILLED, State::OVERFLOW, detectedOverflow,
       entryActions::handleOverflow, nullptr},
      {State::FULL, State::OVERFLOW, detectedOverflow,
       entryActions::handleOverflow, nullptr},
      // Overflow Resolution (Exit: Clear Alarm)
      {State::OVERFLOW, State::DRAINING, overflowResolved,
       entryActions::startDraining, exitActions::clearAlarm},

      // Filling Process
      {State::EMPTY, State::START_FILLING, startFillingCondition,
       entryActions::emptyToFilling, nullptr},
      {State::START_FILLING, State::EMPTY, fillingTimeout, entryActions::stop,
       nullptr},
      {State::START_FILLING, State::FILLING, waterDetected, nullptr,
       nullptr}, // Corrected entry
      {State::FILLING, State::PARTIAL_FILLED, stopCondition, entryActions::stop,
       nullptr},
      {State::FILLING, State::PARTIAL_FILLED, reachedPartial,
       entryActions::stop, nullptr},
      {State::FILLING, State::FULL, reachedFull, entryActions::stop, nullptr},
      {State::PARTIAL_FILLED, State::FULL, reachedFull, entryActions::stop,
       nullptr},

      // Draining Process
      {State::FULL, State::DRAINING, startDrainingCondition,
       entryActions::fullToDraining, nullptr},
      {State::DRAINING, State::EMPTY, reachedEmpty, entryActions::stop,
       nullptr},
      {State::DRAINING, State::PARTIAL_FILLED, stopCondition,
       entryActions::stop, nullptr},
      {State::DRAINING, State::PARTIAL_FILLED, reachedPartial,
       entryActions::stop, nullptr},

      // Partial Filled Transitions
      {State::PARTIAL_FILLED, State::FILLING, startFillingCondition,
       entryActions::partialToFilling, nullptr},
      {State::PARTIAL_FILLED, State::DRAINING, startDrainingCondition,
       entryActions::partialToDraining, nullptr},
  }};

  static constexpr auto transitions_by_state =
      SM::group_transitions_by_state<Transition, transition_count, state_count>(
          transitions);
};

}; // namespace WaterLevel
namespace SM {
class WaterLevelSM : public StateMachine<WaterLevel::Traits, WaterLevelSM> {

public:
  using Traits = WaterLevel::Traits;
  using Context = Traits::Context;
  using State = Traits::State;
  using Inputs = WaterLevel::Inputs;
  using Event = WaterLevel::Event;

  WaterLevelSM(const Context &context = Context{})
      : StateMachine(context, State::EMPTY) {}

  void updateInternalState(const Inputs &input) {
    ctx_.data.current_level = input.sensors.measured_level;
    ctx_.data.current_target_level = input.user_command.new_target_level;
    ctx_.inputs = input;
  }
  void updateInternalState(const Event ev) { ctx_.event = ev; }

  String getStateString() const {
    return GPSU::Util::ToString::WLState(current());
  }
};

} // namespace SM
