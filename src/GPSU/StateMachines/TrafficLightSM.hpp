// TrafficLightSM.hpp
#pragma once
#include "Arduino.h"
#include "StateMachines/StateDefines.hpp"
#include "StateMachines/StateMachine.hpp"
#include "Utility/gpsuUtility.hpp"

namespace TrafficLight {

struct Traits {
  using Context = TrafficLight::Context;
  using Config = TrafficLight::Config;
  using Data = TrafficLight::Data;
  using Inputs = TrafficLight::Inputs;
  using Event = TrafficLight::Event;
  using State = TrafficLight::State;
  using Command = TrafficLight::Command;
  using Mode = TrafficLight::Mode;
  using Guard = bool (*)(const Context &);
  using ExitAction = Command (*)(const Context &);
  using EntryAction = Command (*)(const Context &);

  static constexpr bool has_exit_actions = true;
  static constexpr bool has_entry_actions = true;
  static constexpr uint16_t state_count = 5;
  static constexpr uint16_t transition_count = 18;

  struct Transition {
    State from;
    State to;
    Guard condition;
    EntryAction entry_action;
    ExitAction exit_action;
  };

  static inline bool systemFault(const Context &ctx) {
    return ctx.event == Event::LED_FAULT || ctx.event == Event::TIMER_FAULT;
  }
  static inline bool systemFaultCleared(const Context &ctx) {
    return ctx.event == Event::OK;
  }
  static inline bool auto_mode(const Context &ctx) {
    // Serial.printf("Context Mode is %s",
    //               ctx.mode == TrafficLight::Mode::AUTO ? "Auto" : "Unknown");
    return ctx.mode == Mode::AUTO;
  }
  static inline bool modeChanged(const Context &ctx) {

    return ctx.inputs.mode_changed;
  }
  static inline bool start(const Context &ctx) {
    Serial.printf("Context Mode is %s\n",
                  ctx.mode == TrafficLight::Mode::AUTO ? "Auto" : "Unknown");
    return auto_mode(ctx);
  }
  static inline bool manual_red(const Context &ctx) {
    return !auto_mode(ctx) && ctx.inputs.ui.turn_on_red;
  }
  static inline bool manual_yellow(const Context &ctx) {
    return !auto_mode(ctx) && ctx.inputs.ui.turn_on_yellow;
  }

  static inline bool manual_green(const Context &ctx) {
    return !auto_mode(ctx) && ctx.inputs.ui.turn_on_green;
  }

  static inline bool timeoutRed(const Context &ctx) {
    return auto_mode(ctx) &&
           (ctx.inputs.timer.t1_expired || ctx.data.now >= ctx.config.red_to);
  }
  static inline bool timeoutGreen(const Context &ctx) {
    return auto_mode(ctx) &&
           (ctx.inputs.timer.t1_expired || ctx.data.now >= ctx.config.green_to);
  }

  static inline bool manualYellowToRed(const Context &ctx) {
    return !auto_mode(ctx) && (ctx.inputs.ui.turn_on_red) &&
           (ctx.prev == State::GREEN_STATE);
  }
  static inline bool manualYellowToGreen(const Context &ctx) {
    return !auto_mode(ctx) && (ctx.inputs.ui.turn_on_green) &&
           (ctx.prev == State::RED_STATE);
  }
  static inline bool timeoutYellowToRed(const Context &ctx) {
    return auto_mode(ctx) &&
           (ctx.inputs.timer.t1_expired ||
            ctx.data.now >= ctx.config.yellow_to) &&
           (ctx.prev == State::GREEN_STATE);
  }
  static inline bool timeoutYellowToGreen(const Context &ctx) {
    return auto_mode(ctx) &&
           (ctx.inputs.timer.t1_expired ||
            ctx.data.now >= ctx.config.yellow_to) &&
           (ctx.prev == State::RED_STATE);
  }
  static inline bool buttonPress(const Context &ctx) {
    return ctx.inputs.ui.button_pressed;
  }
  struct entryActions {
    static Command initToRed(const Context &ctx) {

      Command cmd;
      cmd.check_entry = true;
      cmd.entry_command = CmdType::TURN_ON_RED;
      cmd.entry_data.timeout1 = auto_mode(ctx) ? ctx.config.red_to : 0;
      Serial.printf("time out set for red %d \n", cmd.entry_data.timeout1);
      cmd.entry_data.immediate_transition = false;
      return cmd;
    }
    static Command redToYellow(const Context &ctx) {
      Command cmd;
      cmd.check_entry = true;
      cmd.entry_command = CmdType::TURN_ON_YELLOW;
      cmd.entry_data.timeout1 = auto_mode(ctx) ? ctx.config.red_to : 0;
      return cmd;
    }
    static Command redToGreen(const Context &ctx) {
      Command cmd;
      cmd.check_entry = true;
      cmd.entry_command = CmdType::TURN_ON_GREEN;
      cmd.entry_data.immediate_transition = true;
      return cmd;
    }
    static Command yellowToGreen(const Context &ctx) {
      Command cmd;
      cmd.check_entry = true;
      cmd.entry_command = CmdType::TURN_ON_GREEN;
      cmd.entry_data.timeout1 = auto_mode(ctx) ? ctx.config.yellow_to : 0;

      return cmd;
    }
    static Command greenToYellow(const Context &ctx) {
      Command cmd;
      cmd.check_entry = true;
      cmd.entry_command = CmdType::TURN_ON_YELLOW;
      cmd.entry_data.timeout1 = auto_mode(ctx) ? ctx.config.green_to : 0;

      return cmd;
    }
    static Command yellowToRed(const Context &ctx) {
      Command cmd;
      cmd.check_entry = true;
      cmd.entry_command = CmdType::TURN_ON_RED;
      cmd.entry_data.timeout1 = auto_mode(ctx) ? ctx.config.yellow_to : 0;
      return cmd;
    }
  };
  struct exitActions {

    static Command redToYellow(const Context &ctx) {

      Command cmd;
      cmd.check_exit = true;
      cmd.exit_command = CmdType::TURN_OFF_RED;
      return cmd;
    }
    static Command redToGreen(const Context &ctx) {

      Command cmd;
      cmd.check_exit = true;
      cmd.exit_command = CmdType::TURN_OFF_RED;
      cmd.exit_data.immediate_transition = false;
      return cmd;
    }
    static Command yellowToGreen(const Context &ctx) {
      Command cmd;
      cmd.check_exit = true;
      cmd.exit_command = CmdType::TURN_OFF_YELLOW;
      return cmd;
    }
    static Command greenToYellow(const Context &ctx) {
      Command cmd;
      cmd.check_exit = true;
      cmd.exit_command = CmdType::TURN_OFF_GREEN;
      return cmd;
    }
    static Command yellowToRed(const Context &ctx) {
      Command cmd;
      cmd.check_exit = true;
      cmd.exit_command = CmdType::TURN_OFF_YELLOW;
      return cmd;
    }
  };

  static constexpr std::array<Transition, transition_count> transitions = {{
      {State::INIT, State::RED_STATE, start, entryActions::initToRed, nullptr},
      {State::INIT, State::RED_STATE, manual_red, entryActions::initToRed,
       nullptr},
      {State::RED_STATE, State::YELLOW_STATE, timeoutRed,
       entryActions::redToYellow, exitActions::redToYellow},
      {State::RED_STATE, State::YELLOW_STATE, manual_yellow,
       entryActions::redToYellow, exitActions::redToYellow},

      {State::YELLOW_STATE, State::GREEN_STATE, timeoutYellowToGreen,
       entryActions::yellowToGreen, exitActions::yellowToGreen},
      {State::YELLOW_STATE, State::GREEN_STATE, manualYellowToGreen,
       entryActions::yellowToGreen, exitActions::yellowToGreen},

      {State::GREEN_STATE, State::YELLOW_STATE, timeoutGreen,
       entryActions::greenToYellow, exitActions::greenToYellow},
      {State::GREEN_STATE, State::YELLOW_STATE, manual_green,
       entryActions::greenToYellow, exitActions::greenToYellow},

      {State::YELLOW_STATE, State::RED_STATE, timeoutYellowToRed,
       entryActions::yellowToRed, exitActions::yellowToRed},
      {State::YELLOW_STATE, State::RED_STATE, manualYellowToRed,
       entryActions::yellowToRed, exitActions::yellowToRed},

      {State::RED_STATE, State::GREEN_STATE, buttonPress,
       entryActions::redToGreen, exitActions::redToGreen},

      {State::RED_STATE, State::SYSTEM_FAULT, systemFault, nullptr, nullptr},
      {State::YELLOW_STATE, State::SYSTEM_FAULT, systemFault, nullptr, nullptr},
      {State::GREEN_STATE, State::SYSTEM_FAULT, systemFault, nullptr, nullptr},
      {State::SYSTEM_FAULT, State::INIT, systemFaultCleared, nullptr, nullptr},
      {State::RED_STATE, State::INIT, modeChanged, nullptr,
       exitActions::redToYellow},
      {State::GREEN_STATE, State::INIT, modeChanged, nullptr,
       exitActions::greenToYellow},
      {State::YELLOW_STATE, State::INIT, modeChanged, nullptr,
       exitActions::yellowToRed},

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
  using Mode = TrafficLight::Mode;
  explicit TrafficLightSM(const Context &context, State state,
                          bool internalTimer = true)
      : StateMachine(context, state, internalTimer),
        use_internal_timer(internalTimer) {
    // ctx_.mode = mode;
    register_callback(transitionCb);
  }
  TrafficLightSM(const Context &context, bool internalTimer = true)
      : TrafficLightSM(context, State::INIT, internalTimer) {}

  TrafficLightSM() : TrafficLightSM(Context{}, State::INIT, true) {}

  void updateInternalState(const Inputs &input) {
    check_ui_inputs(input);
    bool wants_manual = input.ui.manual_mode;
    if ((wants_manual && ctx_.mode != Mode::MANUAL) ||
        (!wants_manual && ctx_.mode != Mode::AUTO)) {

      updateMode();
    }

    // If mode changed during updateMode, handle transitions
    if (mode_changed) {
      // Reset the flag after handling
      mode_changed = false;
      ctx_.inputs.mode_changed = false;
    }

    if (use_internal_timer && ctx_.mode == Mode::AUTO) {
      if (input.timer.t1_expired) {
        ctx_.data.now = 0;
        update();
      }
    } else if (!use_internal_timer && ctx_.mode == Mode::AUTO) {
      ctx_.data.now += input.timer.ext_delta;
      update();
    } else if (ctx_.mode == Mode::MANUAL) {
      update();
    } else {
    }
  }
  void updateInternalState(const Event ev) { ctx_.event = ev; }
  static void transitionCb(State from, State to, Context &ctx) {
    ctx.data.now = 0;
  }

  const char *getStateString() const {
    return GPSU::Util::ToString::TLState(current());
  }

private:
  bool use_internal_timer;
  bool mode_changed = false;

  void check_ui_inputs(const Inputs &input) {

    if (input.ui.turn_on_red) {
      LOG::DEBUG("TL_SM", "TURN RED BTN");
    } else if (input.ui.turn_on_yellow) {
      LOG::DEBUG("TL_SM", "TURN YELLOW BTN");
    } else if (input.ui.turn_on_green) {
      LOG::DEBUG("TL_SM", "TURN GREEN BTN");
    } else if (input.ui.manual_mode) {
      LOG::DEBUG("TL_SM", "MANUAL MODE");
    }
  }

  void updateMode() {
    ctx_.prev_mode = ctx_.mode;
    if (ctx_.inputs.ui.manual_mode) {
      ctx_.mode = Mode::MANUAL;
      disableTimers();

    } else {
      ctx_.mode = Mode::AUTO;
      enableTimer();
    }
    if (ctx_.mode != ctx_.prev_mode) {
      mode_changed = true;
      ctx_.inputs.mode_changed = true;
      LOG::DEBUG("TL_SM", "Mode changed");
    }

    // Reset selected member variable except the Config and ui Inputs
    ctx_.inputs.new_data = false;
    ctx_.inputs.timer = Inputs::Timer{};
    ctx_.event = Event{};
    ctx_.new_cmd = Command{};

    LOG::DEBUG("TL_SM", "Mode updated to %s",
               ctx_.mode == Mode::MANUAL ? "MANUAL" : "AUTO");
  }
};

} // namespace SM
