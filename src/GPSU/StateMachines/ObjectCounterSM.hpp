// ObjectCounter.hpp
#pragma once
#include "Arduino.h"
#include "StateMachines/StateDefines.hpp"
#include "StateMachines/StateMachine.hpp"
#include "Utility/gpsuUtility.hpp"
#include "string.h"

namespace ObjectCounter {

struct Traits {
  using Context = ObjectCounter::Context;
  using Command = ObjectCounter::Command;
  using Config = ObjectCounter::Config;
  using Data = ObjectCounter::Data;
  using Inputs = ObjectCounter::Inputs;
  using Event = ObjectCounter::Event;
  using Mode = ObjectCounter::Mode;
  using State = ObjectCounter::State;

  using Guard = bool (*)(const Context &);
  using ExitAction = Command (*)(const Context &);
  using EntryAction = Command (*)(const Context &);

  static constexpr bool has_exit_actions = true;
  static constexpr bool has_entry_actions = true;
  static constexpr uint16_t state_count = 6;
  static constexpr uint16_t transition_count = 9;

  struct Transition {
    State from;
    State to;
    Guard condition;
    EntryAction entry_action;
    ExitAction exit_action;
  };

  static inline bool checkFault(const Context &ctx) {

    for (uint8_t i = 0; i < ctx.object_count; i++) {
      if (ctx.objects[i].state == Objects::State::FAILED)
        return true;
    }
    return ctx.event == Event::SAFETY_TIMEOUT;
  }
  static inline bool eStop(const Context &ctx) {
    return ctx.event == Event::E_STOP_ACTIVATED;
  }
  static inline bool faultCleared(const Context &ctx) {
    return ctx.event == Event::OK;
  }
  static inline bool reset(const Context &ctx) {
    return ctx.inputs.user_command.reset;
  }
  static inline bool initToReady(const Context &ctx) { return true; }
  static inline bool auto_mode(const Context &ctx) {
    return ctx.mode == Mode::AUTO;
  }
  static inline bool manual_start(const Context &ctx) {
    return (ctx.mode == Mode::MANUAL && ctx.inputs.user_command.start);
    ;
  }

  static inline bool start(const Context &ctx) {
    return auto_mode(ctx) || manual_start(ctx);
  }
  static inline bool stop(const Context &ctx) {
    return ctx.inputs.user_command.stop && ctx.mode == Mode::MANUAL;
  }
  static inline bool objectPlaced(const Context &ctx) {
    for (uint8_t i = 0; i < ctx.object_count; i++) {
      if (ctx.objects[i].placed_in_conveyor)
        return auto_mode(ctx);
    }
    return false; // No objects triggered the sensor
  }
  static inline bool objectSensed(const Context &ctx) {
    for (uint8_t i = 0; i < ctx.object_count; i++) {
      if (ctx.objects[i].triggered_sensor)
        return auto_mode(ctx);
    }
    return false; // No objects triggered the sensor
  }

  static inline bool objectAtPick(const Context &ctx) {
    for (uint8_t i = 0; i < ctx.object_count; i++) {
      if (ctx.objects[i].reached_picker)
        return auto_mode(ctx);
    }
    return false; // No objects reached the picker
  }

  static inline bool runningToIdle(const Context &ctx) {
    return !auto_mode(ctx) && ctx.inputs.user_command.stop;
  }
  // entry Actions
  static inline Command alarm(const Context &ctx) {
    Command cmd{};
    cmd.check_entry = true;
    cmd.entry_command = CommandType::SOUND_ALARM;
    return cmd;
  }
  static inline Command startConveyor(const Context &ctx) {
    Command cmd{};
    cmd.check_entry = true;
    cmd.entry_command = CommandType::START_CONVEYOR;
    cmd.entry_data.timeout1_ms = ctx.config.object_placement_interval_ms;
    return cmd;
  }
  static inline Command newObject(const Context &ctx) {
    Command cmd{};
    cmd.check_entry = true;
    cmd.entry_command = CommandType::NEW_OBJECT;
    return cmd;
  }
  static inline Command triggerSensor(const Context &ctx) {
    Command cmd{};
    cmd.check_entry = true;
    cmd.entry_command = CommandType::TRIGGER_SENSOR;
    return cmd;
  }
  static inline Command activatePicker(const Context &ctx) {
    Command cmd{};
    cmd.check_entry = true;
    cmd.entry_command = CommandType::ACTIVATE_PICKER;
    return cmd;
  }
  // exit actions
  static inline Command clearAlarm(const Context &ctx) {
    Command cmd;
    cmd.check_exit = true;
    cmd.exit_command = CommandType::CLEAR_ALARM;
    cmd.exit_data.alarm = false; // Turn off the alarm
    return cmd;
  }
  static inline Command stopConveyor(const Context &ctx) {
    Command cmd{};
    cmd.check_exit = true;
    cmd.exit_command = CommandType::STOP_CONVEYOR;
    return cmd;
  }

  static constexpr std::array<Transition, transition_count> transitions = {
      {{State::INIT, State::READY, initToReady, nullptr, nullptr},
       {State::INIT, State::FAULT, checkFault, alarm, clearAlarm},
       {State::FAULT, State::RESET, reset, nullptr, nullptr},
       {State::RESET, State::READY, faultCleared, nullptr, nullptr},
       {State::READY, State::RUNNING, start, startConveyor, nullptr},
       {State::RUNNING, State::FAULT, checkFault, alarm, stopConveyor},
       //    {State::RUNNING, State::RUNNING, objectPlaced, newObject, nullptr},
       //    {State::RUNNING, State::RUNNING, objectSensed, triggerSensor,
       //    nullptr}, {State::RUNNING, State::RUNNING, objectAtPick,
       //    activatePicker, nullptr},
       {State::RUNNING, State::E_STOP, eStop, alarm, stopConveyor},
       {State::E_STOP, State::RESET, reset, nullptr, nullptr},
       {State::RUNNING, State::READY, stop, stopConveyor, nullptr}}

  };
  static constexpr auto transitions_by_state =
      SM::group_transitions_by_state<Transition, transition_count, state_count>(
          transitions);
};
}; // namespace ObjectCounter

namespace SM {
class ObjectCounterSM
    : public StateMachine<ObjectCounter::Traits, ObjectCounterSM> {
public:
  using Traits = ObjectCounter::Traits;
  using Context = ObjectCounter::Context;
  using State = ObjectCounter::State;
  using Inputs = ObjectCounter::Inputs;
  using Event = ObjectCounter::Event;
  using Mode = ObjectCounter::Mode;

  explicit ObjectCounterSM(const Context &context, State state)
      : StateMachine(context, state, true, false) {

    register_callback(resetTransitionCallback);
  }

  ObjectCounterSM() : ObjectCounterSM(Context{}, State::INIT) {}
  void updateInternalState(const Inputs &input) {
    ctx_.inputs = input;
    uint32_t currentTime = millis();
    uint32_t delta = currentTime - ctx_.data.timing.current_time_ms;
    ctx_.data.timing.current_time_ms = currentTime;

    if (current() == State::RUNNING) {
      ctx_.data.timing.conveyor_runtime += delta;

      // Update x_position for all active objects
      for (uint8_t i = 0; i < ctx_.object_count; ++i) {
        auto &obj = ctx_.objects[i];
        if (obj.state != Objects::State::PICKED &&
            obj.state != Objects::State::FAILED) {
          uint32_t elapsed_time =
              ctx_.data.timing.conveyor_runtime - obj.data.placement_time_ms;
          obj.x_position_mm =
              (ObjectCounter::Config::conveyer_velocity_mmps * elapsed_time) /
              1000;
          // Clamp to conveyor length
          if (obj.x_position_mm > ObjectCounter::Config::conveyer_length) {
            obj.x_position_mm = ObjectCounter::Config::conveyer_length;
          }
        }
      }
    }
    if (current() == State::RUNNING &&
        ctx_.inputs.timer.internal_timer1_expired) {
      ctx_.addObject(ctx_.data.timing.conveyor_runtime);
      ctx_.inputs.timer.internal_timer1_expired = false;
      ctx_.data.timing.last_placement_time_ms = currentTime;
      ctx_.updateObjects(ctx_.data.timing.conveyor_runtime);

      // Process picking for all objects at picker
      for (uint8_t i = 0; i < ctx_.object_count; i++) {
        auto &obj = ctx_.objects[i];
        if (obj.state == Objects::State::AT_PICKER) {
          if (ctx_.mode == Mode::AUTO) {
            if (ctx_.data.timing.conveyor_runtime >=
                    obj.data.pick_attempt_time_ms &&
                ctx_.data.timing.conveyor_runtime <=
                    obj.data.pick_deadline_time_ms) {
              obj.state = Objects::State::PICKED;
              Serial.printf("Object id= %d  is PICKED \n", obj.id);
              ctx_.data.production.successful_picks++;
            }
          } else { // MANUAL mode
            if (ctx_.inputs.user_command.pick_now) {
              if (currentTime <= obj.data.pick_deadline_time_ms) {
                obj.state = Objects::State::PICKED;
                ctx_.data.production.successful_picks++;

              } else {
                obj.state = Objects::State::FAILED;
                ctx_.data.production.failed_picks++;
              }
            } else if (currentTime > obj.data.pick_deadline_time_ms) {
              obj.state = Objects::State::FAILED;
              ctx_.data.production.failed_picks++;
            }
          }
        }
      }
      // After processing picks
      ctx_.removeCompletedObjects();
      ctx_.event = Event::NONE;
      restartTimer1(
          static_cast<uint64_t>(ctx_.config.object_placement_interval_ms) *
          1000);
    }
  }
  void updateInternalState(const Event ev) { ctx_.event = ev; }
  static void resetTransitionCallback(State from, State to, Context &ctx) {
    if (from != to) {
      ctx.data.timing = {}; // Reset all timing
      ctx.object_count = 0;
      memset(ctx.objects, 0, sizeof(ctx.objects));
    }
  }

private:
};

} // namespace SM
