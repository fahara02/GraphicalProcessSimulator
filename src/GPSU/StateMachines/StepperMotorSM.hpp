// StepperMotorSM.hpp
#pragma once
#include "Arduino.h"
#include "StateMachines/StateDefines.hpp"
#include "StateMachines/StateMachine.hpp"
#include "Utility/gpsuUtility.hpp"
namespace StepperMotor {

struct Context {
  using Config = StepperMotor::Config;
  using Data = StepperMotor::Data;
  using Inputs = StepperMotor::Inputs;
  using Event = StepperMotor::Event;
  using Pulse = StepperMotor::PulseControl;
  State previous_state;
  Config config;
  Data data;
  Inputs inputs;
  Event event;
  Pulse pulse;
};

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
  static constexpr uint16_t transition_count = 10;

  struct Transition {
    State from;
    State to;
    Guard condition;
    EntryAction entry_action;
    ExitAction exit_action;
  };
  static inline bool cw(const Context &ctx) {
    return ctx.inputs.sensors.direction_state;
  }
  static inline bool cw_limit(const Context &ctx) {
    return ctx.inputs.sensors.cw_limit;
  }
  static inline bool ccw_limit(const Context &ctx) {
    return ctx.inputs.sensors.ccw_limit;
  }
  static inline bool move(const Context &ctx) {
    return ctx.inputs.user_command.start_move;
  }
  static inline bool move_cw(const Context &ctx) {
    return move() && cw() && !ccw_limit();
  }
  static inline bool move_ccw(const Context &ctx) {
    return move() && !cw() && !cw_limit();
  }
  struct entryActions {
    Command movingClockWise(const Context &ctx) {
      Command cmd;
      cmd.check_entry = true;
      cmd.check_exit = false;
      cmd.entry_command = CommandType::START_MOVE;
      cmd.entry_data.direction = ctx.inputs.sensors.direction_state;
    }
  };
  struct exitActions {};

  static constexpr std::array<Transition, transition_count> transitions = {
      {State::IDLE, State::MOVING_CW, move_cw, entryActions::initToRed,
       nullptr},
  }
};

} // namespace StepperMotor