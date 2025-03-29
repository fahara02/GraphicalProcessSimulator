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

    for (uint8_t i = 0; i < ctx.obj_cnt; i++) {
      if (ctx.items[i].state == Items::State::FAILED)
        return true;
    }
    return ctx.event == Event::SAFETY_TO;
  }
  static inline bool eStop(const Context &ctx) {
    return ctx.event == Event::ESTOP;
  }
  static inline bool faultCleared(const Context &ctx) {
    return ctx.event == Event::OK;
  }
  static inline bool reset(const Context &ctx) { return ctx.inputs.ui.reset; }
  static inline bool initToReady(const Context &ctx) { return true; }
  static inline bool auto_mode(const Context &ctx) {
    return ctx.mode == Mode::AUTO;
  }
  static inline bool manual_start(const Context &ctx) {
    return (ctx.mode == Mode::MANUAL && ctx.inputs.ui.start);
    ;
  }

  static inline bool start(const Context &ctx) {
    return auto_mode(ctx) || manual_start(ctx);
  }
  static inline bool stop(const Context &ctx) {
    return ctx.inputs.ui.stop && ctx.mode == Mode::MANUAL;
  }
  static inline bool objectPlaced(const Context &ctx) {
    for (uint8_t i = 0; i < ctx.obj_cnt; i++) {
      if (ctx.items[i].on_conv)
        return auto_mode(ctx);
    }
    return false; // No items triggered the sensor
  }
  static inline bool objectSensed(const Context &ctx) {
    for (uint8_t i = 0; i < ctx.obj_cnt; i++) {
      if (ctx.items[i].sensed)
        return auto_mode(ctx);
    }
    return false; // No items triggered the sensor
  }

  static inline bool objectAtPick(const Context &ctx) {
    for (uint8_t i = 0; i < ctx.obj_cnt; i++) {
      if (ctx.items[i].at_pick)
        return auto_mode(ctx);
    }
    return false; // No items reached the picker
  }

  static inline bool runningToIdle(const Context &ctx) {
    return !auto_mode(ctx) && ctx.inputs.ui.stop;
  }
  // entry Actions
  static inline Command alarm(const Context &ctx) {
    Command cmd{};
    cmd.check_entry = true;
    cmd.entry_command = CmdType::ALARM;
    return cmd;
  }
  static inline Command startConveyor(const Context &ctx) {
    Command cmd{};
    cmd.check_entry = true;
    cmd.entry_command = CmdType::START;
    cmd.entry_data.timeout1 = ctx.config.place_interval;
    return cmd;
  }
  static inline Command newObject(const Context &ctx) {
    Command cmd{};
    cmd.check_entry = true;
    cmd.entry_command = CmdType::NEW_OBJ;
    return cmd;
  }
  static inline Command triggerSensor(const Context &ctx) {
    Command cmd{};
    cmd.check_entry = true;
    cmd.entry_command = CmdType::SENSE;
    return cmd;
  }
  static inline Command activatePicker(const Context &ctx) {
    Command cmd{};
    cmd.check_entry = true;
    cmd.entry_command = CmdType::PICK;
    return cmd;
  }
  // exit actions
  static inline Command clearAlarm(const Context &ctx) {
    Command cmd;
    cmd.check_exit = true;
    cmd.exit_command = CmdType::CLEAR_ALARM;
    cmd.exit_data.alarm = false; // Turn off the alarm
    return cmd;
  }
  static inline Command stopConveyor(const Context &ctx) {
    Command cmd{};
    cmd.check_exit = true;
    cmd.exit_command = CmdType::STOP;
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
  using Item = ObjectCounter::Item;

  explicit ObjectCounterSM(const Context &context, State state)
      : StateMachine(context, state, true, false) {

    register_callback(transitionCb);
  }

  ObjectCounterSM() : ObjectCounterSM(Context{}, State::INIT) {}

  void updateInternalState(const Inputs &input) {
    ctx_.inputs = input;
    uint32_t currentTime = millis();
    uint32_t delta = currentTime - ctx_.data.timing.now;
    ctx_.data.timing.now = currentTime;

    if (current() == State::RUNNING) {
      ctx_.data.timing.runtime += delta;
      updateObjects(ctx_.data.timing.runtime);
      simulateSensor();

      // Process picking for all items at picker
      for (uint8_t i = 0; i < ctx_.obj_cnt; i++) {
        auto &obj = ctx_.items[i];
        if (obj.state == Items::State::ARRIVAL) {
          if (ctx_.mode == Mode::AUTO) {
            if (ctx_.data.timing.runtime >= obj.data.pick_attempt &&
                ctx_.data.timing.runtime <= obj.data.deadline) {
              obj.state = Items::State::PICKED;
              LOG::DEBUG("Item id= %d  is PICKED \n", obj.id);
              ctx_.data.stat.ok++;
            }
          } else { // MANUAL mode
            if (ctx_.inputs.ui.pick && currentTime <= obj.data.deadline) {
              obj.state = Items::State::PICKED;
              ctx_.data.stat.ok++;
            } else if (currentTime > obj.data.deadline) {
              obj.state = Items::State::FAILED;
              ctx_.data.stat.failed++;
            }
          }
        }
      }
      removeCompleted();

      if (ctx_.inputs.timer.t1_expired) {
        addObject(ctx_.data.timing.runtime);
        ctx_.inputs.timer.t1_expired = false;
        ctx_.data.timing.last_placed = currentTime;
        restartTimer1(static_cast<uint64_t>(ctx_.config.place_interval) * 1000);
      }
    }
  }
  void updateInternalState(const Event ev) { ctx_.event = ev; }
  static void transitionCb(State from, State to, Context &ctx) {
    if (from != to) {
      ctx.data.timing = {}; // Reset all timing
      ctx.obj_cnt = 0;
      memset(ctx.items, 0, sizeof(ctx.items));
    }
  }

private:
  void addObject(uint32_t runtime) {
    if (ctx_.obj_cnt >= Context::Config::max_objs)
      return;

    Item item;
    item.id = ctx_.id_cnt;
    item.state = Items::State::PLACED;
    item.on_conv = true;
    item.data = {runtime, runtime + ctx_.config.sense_delay,
                 runtime + ctx_.config.pick_delay, 0, 0};

    ctx_.items[ctx_.obj_cnt++] = item;
    ctx_.id_cnt++;
    LOG::DEBUG("OC_SM", "Item id= %d placed\n", item.id);
  }

  void updateObjects(uint32_t runtime) {
    for (uint8_t i = 0; i < ctx_.obj_cnt; i++) {
      auto &item = ctx_.items[i];

      // Update x_position
      if (item.state != Items::State::PICKED &&
          item.state != Items::State::FAILED) {
        uint32_t elapsed_time = runtime - item.data.place_time;
        item.x_pos = (ObjectCounter::Config::conv_mmps * elapsed_time) / 1000;
        if (item.x_pos > ObjectCounter::Config::conv_length) {
          item.x_pos = ObjectCounter::Config::conv_length;
        }
      }

      // Check for SENSED transition

      if (!item.sensed) {

        if (ctx_.mode == Mode::MANUAL) {
          if (runtime >= item.data.sense_time) {
            item.state = Items::State::SENSED;
            item.sensed = true;
            ctx_.event = Event::SENSED;
            ctx_.inputs.sensors.photoeye = true;
            LOG::DEBUG("OC_SM", "Item %d sensed\n", item.id);
          }
        } else {
          if (runtime >= item.data.sense_time) {
            item.state = Items::State::SENSED;
            item.sensed = true;
            ctx_.event = Event::SENSED;
            LOG::DEBUG("OC_SM", "Item %d sensed\n", item.id);
          }
        }
      }
      if (!item.sensed && runtime >= item.data.sense_time) {
        item.state = Items::State::SENSED;
        item.sensed = true;
        ctx_.event = Event::SENSED;
        LOG::DEBUG("OC_SM", "Item %d sensed\n", item.id);
      }

      // Check for ARRIVAL transition
      if (!item.at_pick && runtime >= item.data.pick_time) {
        item.state = Items::State::ARRIVAL;
        item.at_pick = true;
        ctx_.event = Event::ARRIVAL;
        pickerProcessing(item, runtime);
        LOG::DEBUG("OC_SM", "Item %d at picker\n", item.id);
      }
    }
  }
  void pickerProcessing(Item &obj, uint32_t runtime) {
    obj.data.pick_attempt =
        runtime + (ctx_.mode == Mode::AUTO ? ctx_.config.sim_pick_delay : 0);

    obj.data.deadline =
        runtime + (ctx_.mode == Mode::AUTO ? ctx_.config.auto_timeout
                                           : ctx_.config.manual_timeout);
  }
  void removeCompleted() {
    uint8_t newCount = 0;
    for (uint8_t i = 0; i < ctx_.obj_cnt; ++i) {
      if (ctx_.items[i].state == Items::State::PICKED ||
          ctx_.items[i].state == Items::State::FAILED) {
        LOG::DEBUG("OC_SM", "Removing item %d\n", ctx_.items[i].id);
      } else {
        ctx_.items[newCount++] = ctx_.items[i];
      }
    }
    ctx_.obj_cnt = newCount;
  }
  void simulateSensor() {
    if (ctx_.inputs.sensors.photoeye) {
    }
  }
};

} // namespace SM
