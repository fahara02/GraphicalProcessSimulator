// ObjectCounter.hpp
#pragma once
#include "Arduino.h"
#include "StateMachines/StateDefines.hpp"
#include "StateMachines/StateMachine.hpp"
#include "Utility/gpsuUtility.hpp"
namspace ObjectCounter {

  struct Context {
    using Config = ObjectCounter::Config;
    using Data = ObjectCounter::Data;
    using Inputs = ObjectCounter::Inputs;
    using Event = ObjectCounter::Event;
    using Mode = ObjectCounter::Mode;

    State previous_state;
    Config config;
    Data data;
    Inputs inputs;
    Event event;
    Mode mode = Mode::AUTO;

    // Assignment operator
    Context &operator=(const Context &other) {
      if (this != &other) {
        previous_state = other.previous_state;
        config = other.config;
        data = other.data;
        inputs = other.inputs;
        event = other.event;
        mode = other.mode;
      }
      return *this;
    }
  };

  struct Traits {
    using Context = ObjectCounter::Context;
    using Config = ObjectCounter::Config;
    using Data = ObjectCounter::Data;
    using Inputs = ObjectCounter::Inputs;
    using Event = ObjectCounter::Event;
    using State = ObjectCounter::State;
    using Command = ObjectCounter::Command;
    using Mode = ObjectCounter::Mode;
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

    static inline bool systemFault(const Context &ctx) {
      return ctx.event == Event::SENSOR_RUNWAY ||
             ctx.event == Event::E_STOP_ACTIVATED;
    }
    static inline bool systemFaultCleared(const Context &ctx) {
      return ctx.event == Event::OK;
    }
    static inline bool auto_mode(const Context &ctx) {
      return ctx.mode == Mode::AUTO;
    }
    static inline bool manual_start(const Context &ctx) {
      return ctx.inputs.user_command.button_pressed;
    }
    static inline bool initToReady(const Context &ctx) {}

    static inline bool readyToRunning(const Context &ctx) {
      return auto_mode(ctx) || manual_start(ctx);
    }

    static inline bool ruuningToOP(const Context &ctx) {
      return ctx.inputs.timer.current_placement_time_ms >=
             ctx.config.per_object_time_msec;
    }
    static inline bool arrivedOnSensePoint(const Context &ctx) {
      // will trigger a sensor relay
      return ctx.inputs.timer.current_placed_object_ms >=
             ctx.config.object_arrival_duration_sp_ms;
    }

    static inline bool objectPlacedToObjectSensed(const Context &ctx) {
      return auto_mode(ctx) && arrivedOnSensePoint(ctx) ||
             !auto_mode(ctx) && ctx.inputs.new_input &&
                 ctx.inputs.acknowledge_sensor; // Simulated sensor trigger will
                                                // sensed by users plc
    }
    static inline bool arrivedOnPickPoint(const Context &ctx) {
      // will trigger a sensor relay
      return ctx.inputs.timer.current_placed_object_ms >=
             ctx.config.object_arrival_duration_pp_ms;
    }
    static inline bool objectSensedToPicking(const Context &ctx) {
      return auto_mode(ctx) && arrivedOnPickPoint(ctx) ||
             !auto_mode(ctx) && arrivedOnPickPoint(ctx) &&
                 ctx.inputs.new_input &&
                 ctx.inputs.pick_arrived; // user plc input to confirm user
                                          // knows its picking
    }
    static inline bool pickingToPick(const Context &ctx) {
      // will trigger a pick  relay
      return (auto_mode(ctx) && ctx.inputs.timer.current_placed_object_ms >=
                                    ctx.config.auto_picking_time_ms) ||
             (!auto_mode(ctx) && ctx.inputs.new_input && ctx.inputs.pick_now;)
    }
    static inline bool pickingToPickFailed(const Context &ctx) {
      // will trigger alarm relay
      return (auto_mode(ctx) && false) ||
             (!auto_mode(ctx) && ctx.inputs.timer.current_placed_object_ms >=
                                     ctx.config.picking_time_limit_ms;)
    }

    static inline bool objectOutToRunning(const Context &ctx) {
      return true; // Immediate transition
    }

    static inline bool runningToIdle(const Context &ctx) {
      return !auto_mode(ctx) && ctx.inputs.stop;
    }
    struct entryActions {};
    struct exitActions {};
  }
}