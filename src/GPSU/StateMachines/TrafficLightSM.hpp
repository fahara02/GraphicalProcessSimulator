// TrafficLightSM.hpp
#pragma once
#include "Arduino.h"
#include "StateMachines/StateDefines.hpp"
#include "StateMachines/StateMachine.hpp"
#include "Utility/gpsuUtility.hpp"

namespace TrafficLight {

struct Context {
  using Config = TrafficLight::Config;
  using Data = TrafficLight::Data;
  using Inputs = TrafficLight::Inputs;
  using Event = TrafficLight::Event;
  State previous_state;
  Config config;
  Data data;
  Inputs inputs;
  Event event;
};

struct Traits {
  using Context = TrafficLight::Context;
  using Config = TrafficLight::Config;
  using Data = TrafficLight::Data;
  using Inputs = TrafficLight::Inputs;
  using Event = TrafficLight::Event;
  using State = TrafficLight::State;
  using Command = TrafficLight::Command;
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

  static inline bool alwaysTrue(const Context &ctx) { return true; }
  static inline bool systemFault(const Context &ctx) {
    return ctx.event == Event::LED_FAULT || ctx.event == Event::TIMER_FAULT;
  }
  static inline bool systemFaultCleared(const Context &ctx) {
    return ctx.event == Event::OK;
  }

  static inline bool timeoutRed(const Context &ctx) {
    return ctx.data.current_time_ms >= ctx.config.redTimeout_ms;
  }
  static inline bool timeoutGreen(const Context &ctx) {
    return ctx.data.current_time_ms >= ctx.config.greenTimeout_ms;
  }
  static inline bool timeoutYellowToRed(const Context &ctx) {
    return (ctx.data.current_time_ms >= ctx.config.yellowTimeout_ms) &&
           (ctx.previous_state == State::GREEN_STATE);
  }
  static inline bool timeoutYellowToGreen(const Context &ctx) {
    return (ctx.data.current_time_ms >= ctx.config.yellowTimeout_ms) &&
           (ctx.previous_state == State::RED_STATE);
  }
  static inline bool buttonPress(const Context &ctx) {
    return ctx.inputs.user_command.button_pressed;
  }
  struct entryActions {
    static Command initToRed(const Context &ctx) {
      Command cmd;
      cmd.check_entry = true;
      cmd.check_exit = false;
      cmd.entry_command = CommandType::TURN_ON_RED;
      cmd.entry_data.timeout_ms = 0;
      cmd.entry_data.immediate_transition = false;
      return cmd;
    }
    static Command redToYellow(const Context &ctx) {

      Command cmd;
      cmd.check_entry = true;
      cmd.check_exit = false;
      cmd.entry_command = CommandType::TURN_ON_YELLOW;
      cmd.entry_data.timeout_ms = ctx.config.yellowTimeout_ms;
      return cmd;
    }
    static Command redToGreen(const Context &ctx) {

      Command cmd;
      cmd.check_entry = true;
      cmd.check_exit = false;
      cmd.entry_command = CommandType::TURN_ON_GREEN;
      cmd.entry_data.immediate_transition = true;
      return cmd;
    }
    static Command yellowToGreen(const Context &ctx) {
      Command cmd;
      cmd.check_entry = true;
      cmd.check_exit = false;
      cmd.entry_command = CommandType::TURN_ON_GREEN;
      cmd.entry_data.timeout_ms = ctx.config.greenTimeout_ms;

      return cmd;
    }
    static Command greenToYellow(const Context &ctx) {
      Command cmd;
      cmd.check_entry = true;
      cmd.check_exit = false;
      cmd.entry_command = CommandType::TURN_ON_YELLOW;
      cmd.entry_data.timeout_ms = ctx.config.yellowTimeout_ms;

      return cmd;
    }
    static Command yellowToRed(const Context &ctx) {
      Command cmd;
      cmd.check_entry = true;
      cmd.check_exit = false;
      cmd.entry_command = CommandType::TURN_ON_RED;
      cmd.entry_data.timeout_ms = ctx.config.redTimeout_ms;
      return cmd;
    }
  };
  struct exitActions {

    static Command redToYellow(const Context &ctx) {

      Command cmd;
      cmd.check_entry = false;
      cmd.check_exit = true;
      cmd.exit_command = CommandType::TURN_OFF_RED;
      return cmd;
    }
    static Command redToGreen(const Context &ctx) {

      Command cmd;
      cmd.check_entry = false;
      cmd.check_exit = true;
      cmd.exit_command = CommandType::TURN_OFF_RED;
      cmd.exit_data.immediate_transition = false;
      return cmd;
    }
    static Command yellowToGreen(const Context &ctx) {
      Command cmd;
      cmd.check_entry = false;
      cmd.check_exit = true;
      cmd.exit_command = CommandType::TURN_OFF_YELLOW;
      return cmd;
    }
    static Command greenToYellow(const Context &ctx) {
      Command cmd;
      cmd.check_entry = false;
      cmd.check_exit = true;
      cmd.exit_command = CommandType::TURN_OFF_GREEN;
      return cmd;
    }
    static Command yellowToRed(const Context &ctx) {
      Command cmd;
      cmd.check_entry = false;
      cmd.check_exit = true;
      cmd.exit_command = CommandType::TURN_OFF_YELLOW;
      return cmd;
    }
  };

  static constexpr std::array<Transition, transition_count> transitions = {{
      {State::INIT, State::RED_STATE, alwaysTrue, entryActions::initToRed,
       nullptr},
      {State::RED_STATE, State::YELLOW_STATE, timeoutRed,
       entryActions::redToYellow, exitActions::redToYellow},
      {State::YELLOW_STATE, State::GREEN_STATE, timeoutYellowToGreen,
       entryActions::yellowToGreen, exitActions::yellowToGreen},
      {State::GREEN_STATE, State::YELLOW_STATE, timeoutGreen,
       entryActions::greenToYellow, exitActions::greenToYellow},
      {State::YELLOW_STATE, State::RED_STATE, timeoutYellowToRed,
       entryActions::yellowToRed, exitActions::yellowToRed},
      {State::RED_STATE, State::GREEN_STATE, buttonPress,
       entryActions::redToGreen, exitActions::redToGreen},
      {State::RED_STATE, State::SYSTEM_FAULT, systemFault, nullptr, nullptr},
      {State::YELLOW_STATE, State::SYSTEM_FAULT, systemFault, nullptr, nullptr},
      {State::GREEN_STATE, State::SYSTEM_FAULT, systemFault, nullptr, nullptr},
      {State::SYSTEM_FAULT, State::INIT, systemFaultCleared, nullptr, nullptr},

  }

  };

  static constexpr auto transitions_by_state =
      SM::group_transitions_by_state<Transition, transition_count, state_count>(
          transitions);
};

} // namespace TrafficLight

namespace SM {

class TrafficLightSM
    : public StateMachine<TrafficLight::Traits, TrafficLightSM> {
public:
  using Traits = TrafficLight::Traits;
  using Context = Traits::Context;
  using State = Traits::State;
  using Inputs = TrafficLight::Inputs;
  using Event = TrafficLight::Event;
  explicit TrafficLightSM(const Context &context, State state)
      : StateMachine(context, state) {

    register_callback(resetTimerCallback);
  }
  TrafficLightSM(const Context &context)
      : TrafficLightSM(context, State::INIT) {}
  TrafficLightSM() : TrafficLightSM(Context{}, State::INIT) {}

  void updateInternalState(const Inputs &input) {
    ctx_.data.current_time_ms += input.external_timer.delta_time_ms;
    ctx_.inputs = input;
  }
  void updateInternalState(const Event ev) { ctx_.event = ev; }
  static void resetTimerCallback(State from, State to, Context &ctx) {
    ctx.data.current_time_ms = 0;
  }

  const char *getStateString() const {
    return GPSU::Util::ToString::TLState(current());
  }
};

} // namespace SM
