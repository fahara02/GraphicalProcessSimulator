// StepperMotorSM.hpp
#pragma once
#include "Arduino.h"
#include "StateMachines/StateDefines.hpp"
#include "StateMachines/StateMachine.hpp"
#include "Utility/gpsuUtility.hpp"
namespace StepperMotor {



struct Traits {
  using Context = StepperMotor::Context;
  using Config = StepperMotor::Config;
  using Data = StepperMotor::Data;
  using Inputs = StepperMotor::Inputs;
  using Event = StepperMotor::Event;
  using State = StepperMotor::State;
  using Command = StepperMotor::Command;
  using Guard = bool (*)(const Context &);
  using ExitAction = Command (*)(const Context &);
  using EntryAction = Command (*)(const Context &);

  static constexpr bool has_exit_actions = true;
  static constexpr bool has_entry_actions = true;
  static constexpr uint16_t state_count = 5;
  static constexpr uint16_t transition_count = 15;

  struct Transition {
    State from;
    State to;
    Guard condition;
    EntryAction entry_action;
    ExitAction exit_action;
  };
  static inline bool move(const Context &ctx) {
    return ctx.inputs.user_command.start_move;
  }
  static inline bool cw(const Context &ctx) {
    return ctx.inputs.sensors.direction_state;
  }
  static inline bool cw_limit(const Context &ctx) {
    return ctx.inputs.sensors.cw_limit;
  }
  static inline bool ccw_limit(const Context &ctx) {
    return ctx.inputs.sensors.ccw_limit;
  }
  static inline bool home(const Context &ctx) {
    return ctx.inputs.user_command.home_request;
  }
  static inline bool move_cw(const Context &ctx) {
    return move(ctx) && cw(ctx) && !ccw_limit(ctx);
  }
  static inline bool move_ccw(const Context &ctx) {
    return move(ctx) && !cw(ctx) && !cw_limit(ctx);
  }
  static inline bool fault_detected(const Context &ctx) {
    return ctx.inputs.sensors.driver_fault || ctx.inputs.sensors.over_temp;
  }
  static inline bool movement_complete(const Context &ctx) {
    return abs(ctx.data.current_position - ctx.data.target_position) <=
           ctx.config.position_tolerance;
  }
  static inline bool stop_requested(const Context &ctx) {
    return ctx.inputs.user_command.stop;
  }
  static inline bool home_complete(const Context &ctx) {
    return ctx.inputs.sensors.home_switch;
  }
  static inline bool clear_error(const Context &ctx) {
    return ctx.inputs.user_command.clear_error;
  }
  struct entryActions {
    static Command start_move_cw(const Context &ctx) {
      Command cmd;
      cmd.check_entry = true;
      cmd.entry_command = CommandType::START_MOVE;
      cmd.entry_data.direction = true; // CW
      cmd.entry_data.speed_rpm = ctx.config.max_rpm;
      cmd.entry_data.target_position = ctx.inputs.user_command.target_position;
      return cmd;
    }

    static Command start_move_ccw(const Context &ctx) {
      Command cmd;
      cmd.check_entry = true;
      cmd.entry_command = CommandType::START_MOVE;
      cmd.entry_data.direction = false; // CCW
      cmd.entry_data.speed_rpm = ctx.config.max_rpm;
      cmd.entry_data.target_position = ctx.inputs.user_command.target_position;
      return cmd;
    }

    static Command start_homing(const Context &ctx) {
      Command cmd;
      cmd.check_entry = true;
      cmd.entry_command = CommandType::HOME;
      cmd.entry_data.direction = false;                  // Move CCW to home
      cmd.entry_data.speed_rpm = ctx.config.max_rpm / 2; // Slower homing speed
      return cmd;
    }

    static Command emergency_stop(const Context &ctx) {
      Command cmd;
      cmd.check_entry = true;
      cmd.entry_command = CommandType::EMERGENCY_STOP;
      return cmd;
    }
  };

  struct exitActions {
    static Command stop_motor(const Context &ctx) {
      Command cmd;
      cmd.check_exit = true;
      cmd.exit_command = CommandType::STOP;
      return cmd;
    }
  };

  static constexpr std::array<Transition, transition_count> transitions = {{
      // From IDLE
      {State::IDLE, State::MOVING_CW, move_cw, entryActions::start_move_cw,
       exitActions::stop_motor},
      {State::IDLE, State::MOVING_CCW, move_ccw, entryActions::start_move_ccw,
       exitActions::stop_motor},
      {State::IDLE, State::HOMING, home, entryActions::start_homing,
       exitActions::stop_motor},
      {State::IDLE, State::ERROR, fault_detected, entryActions::emergency_stop,
       nullptr},

      // From MOVING_CW
      {State::MOVING_CW, State::IDLE, movement_complete, nullptr,
       exitActions::stop_motor},
      {State::MOVING_CW, State::IDLE, stop_requested, nullptr,
       exitActions::stop_motor},
      {State::MOVING_CW, State::ERROR, fault_detected,
       entryActions::emergency_stop, exitActions::stop_motor},
      {State::MOVING_CW, State::ERROR, ccw_limit, entryActions::emergency_stop,
       exitActions::stop_motor},

      // From MOVING_CCW
      {State::MOVING_CCW, State::IDLE, movement_complete, nullptr,
       exitActions::stop_motor},
      {State::MOVING_CCW, State::IDLE, stop_requested, nullptr,
       exitActions::stop_motor},
      {State::MOVING_CCW, State::ERROR, fault_detected,
       entryActions::emergency_stop, exitActions::stop_motor},
      {State::MOVING_CCW, State::ERROR, cw_limit, entryActions::emergency_stop,
       exitActions::stop_motor},

      // From HOMING
      {State::HOMING, State::IDLE, home_complete, nullptr,
       exitActions::stop_motor},
      {State::HOMING, State::ERROR, fault_detected,
       entryActions::emergency_stop, exitActions::stop_motor},

      // From ERROR
      {State::ERROR, State::IDLE, clear_error, nullptr, nullptr},
  }};

  static constexpr auto transitions_by_state =
      SM::group_transitions_by_state<Transition, transition_count, state_count>(
          transitions);
};

} // namespace StepperMotor
namespace SM {

class StepperMotorSM
    : public StateMachine<StepperMotor::Traits, StepperMotorSM> {
public:
  using Traits = StepperMotor::Traits;
  using Context = Traits::Context;
  using State = Traits::State;
  using Inputs = StepperMotor::Inputs;
  using Event = StepperMotor::Event;

  explicit StepperMotorSM(const Context &context, State state)
      : StateMachine(context, state, false) {}

  StepperMotorSM(const Context &context)
      : StepperMotorSM(context, State::IDLE) {}

  StepperMotorSM() : StepperMotorSM(Context{}, State::IDLE) {}

  void updateInternalState(const Inputs &input) {
    // Update position based on encoder pulses
    ctx_.data.current_position +=
        input.sensors.pulse_count * (input.sensors.direction_state ? 1 : -1);
    ctx_.inputs = input;
  }

  void updateInternalState(const Event ev) { ctx_.event = ev; }
};

} // namespace SM