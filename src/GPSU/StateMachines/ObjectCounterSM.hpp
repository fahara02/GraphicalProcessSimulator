// ObjectCounter.hpp
#pragma once
#include "Arduino.h"
#include "MCP23017.hpp"
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
    return (ctx.mode == Mode::MANUAL && ctx.inputs.ui.start &&
            !ctx.inputs.ui.stop);
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
    return !auto_mode(ctx) && !ctx.inputs.ui.start;
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
  using IO = COMPONENT::MCP23017;

  explicit ObjectCounterSM(const Context &context, State state,
                           std::shared_ptr<COMPONENT::MCP23017> io)
      : StateMachine(context, state, true, false), io_(io) {
    register_callback(transitionCb);
  }

  ObjectCounterSM(std::shared_ptr<COMPONENT::MCP23017> io)
      : ObjectCounterSM(Context{}, State::INIT, io) {}

  void updateInternalState(const Inputs &input) {
    check_ui_inputs();

    // Detect mode change before calling updateMode
    bool mode_changed =
        (ctx_.inputs.ui.manual_mode ? Mode::MANUAL : Mode::AUTO) != ctx_.mode;

    updateMode(ctx_.inputs.ui.manual_mode);

    uint32_t currentTime = millis();
    uint32_t delta = currentTime - ctx_.data.timing.now;
    ctx_.data.timing.now = currentTime;

    // Handle timer restart on mode change to MANUAL
    if (mode_changed && ctx_.mode == Mode::MANUAL &&
        current() == State::RUNNING) {
      restartTimer1(ctx_.config.place_interval * 1000);
      LOG::DEBUG("OC_SM", "Restarted placement timer for manual mode");
    }

    if (current() == State::RUNNING) {
      ctx_.data.timing.runtime += delta;
      updateObjects(ctx_.data.timing.runtime);
      removeFailed();

      // Handle timer-based object placement (works for both modes)
      if (ctx_.inputs.timer.t1_expired) {
        addObject(ctx_.data.timing.runtime);
        ctx_.inputs.timer.t1_expired = false;
        ctx_.data.timing.last_placed = currentTime;
        restartTimer1(ctx_.config.place_interval * 1000);
      }
    }

    last_mode_ = ctx_.mode; // Update mode tracking
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
  Mode last_mode_ = Mode::AUTO;
  std::shared_ptr<COMPONENT::MCP23017> io_;
  void updateMode(bool manual_requested) {
    const Mode new_mode = manual_requested ? Mode::MANUAL : Mode::AUTO;
    ctx_.prev_mode = ctx_.mode;
    if (new_mode != ctx_.mode) {

      ctx_.mode = new_mode;
      ctx_.inputs.mode_changed = true;
      LOG::DEBUG("SM", "Mode updated to %s",
                 ctx_.mode == Mode::MANUAL ? "MANUAL" : "AUTO");
      if (ctx_.mode == Mode::AUTO) {

        if (timer1_was_deleted) {
          timer1_was_deleted = false;
          enableTimer(0);
          createTimers();

        } else if (timer2_was_deleted) {
          timer2_was_deleted = false;
          enableTimer(1);
          createTimers();
        }
      } else {
        disableTimers();
        enableTimer(0);
        createTimers();
      }
      setAutoUpdate();
      update();
      ctx_.inputs.mode_changed = false;
    }
  }
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
      } else if (item.state == Items::State::PICKED) {
        uint32_t elapsed_time = runtime - item.data.place_time;
        item.y_pos += 2;
        if (item.y_pos >= 25) {
          removeItem(item.id);
        }
      }

      if (!item.sensed) {
        // manual and auto both will have same
        if (runtime >= item.data.sense_time) {
          item.state = Items::State::SENSED;
          item.sensed = true;
          ctx_.event = Event::SENSED;
          ctx_.inputs.sensors.photoeye = true;
          LOG::DEBUG("OC_SM", "Item %d sensed\n", item.id);
        } else if (item.state != Items::State::SENSED ||
                   runtime >= ctx_.config.sense_timeout) {
          ctx_.inputs.sensors.photoeye = false;
        }
      }
      simulateSensor(item.id);
      // Check for ARRIVAL transition
      if (!item.at_pick && runtime >= item.data.pick_time) {
        item.state = Items::State::ARRIVAL;
        item.at_pick = true;
        ctx_.event = Event::ARRIVAL;
        pickerProcessing(item, runtime);
        LOG::DEBUG("OC_SM", "Item %d at picker\n", item.id);
      }
      bool pick_state = false;
      if (item.state == Items::State::ARRIVAL) {
        if (ctx_.mode == Mode::AUTO) {
          if (ctx_.data.timing.runtime >= item.data.pick_attempt &&
              ctx_.data.timing.runtime <= item.data.deadline) {
            item.state = Items::State::PICKED;
            trigger_pick(item.id, true);
            LOG::DEBUG("OC_SM", "Item id= %d  is PICKED \n", item.id);
            ctx_.data.stat.ok++;
          }
        } else { // MANUAL mode
          if (ctx_.inputs.ui.pick && runtime <= item.data.deadline) {
            item.state = Items::State::PICKED;
            trigger_pick(item.id, true);
            ctx_.data.stat.ok++;
            LOG::DEBUG("OC_SM", "Item id= %d  is Manual PICKED \n", item.id);
          } else if (runtime > item.data.deadline) {
            item.state = Items::State::FAILED;
            LOG::DEBUG("OC_SM", "Item id= %d  is failed to  PICK \n", item.id);
            ctx_.data.stat.failed++;
          }
        }
      }
      if (ctx_.mode == Mode::AUTO && runtime >= ctx_.config.pick_timeout) {
        trigger_pick(item.id, false);
      } else if (ctx_.mode == Mode::MANUAL && !ctx_.inputs.ui.pick) {
        trigger_pick(item.id, false);
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
  void removeFailed() {
    uint8_t newCount = 0;
    for (uint8_t i = 0; i < ctx_.obj_cnt; ++i) {
      if (ctx_.items[i].state == Items::State::FAILED) {
        LOG::DEBUG("OC_SM", "After Failed Removing item %d\n",
                   ctx_.items[i].id);
      } else {
        ctx_.items[newCount++] = ctx_.items[i];
      }
    }
    ctx_.obj_cnt = newCount;
  }
  void removeItem(int id) {
    uint8_t newCount = 0;
    for (uint8_t i = 0; i < ctx_.obj_cnt; ++i) {
      if (ctx_.items[i].id == id) {
        LOG::DEBUG("OC_SM", "After Pick Removingitem %d\n", ctx_.items[i].id);
      } else {
        ctx_.items[newCount++] = ctx_.items[i];
      }
    }
    ctx_.obj_cnt = newCount;
  }
  void simulateSensor(int id) {
    bool level = ctx_.inputs.sensors.photoeye;
    io_->digitalWrite(GPA1, level);
    LOG::TEST("OC_SM", "Simulating sensor Item %d sensed\n", id);
  }
  void trigger_pick(int id, bool level) {
    io_->digitalWrite(GPA2, level);
    LOG::TEST("OC_SM", "Simulating pick Item %d sensed\n", id);
  }

  void check_ui_inputs() {
    uint8_t pinStatus = 0;
    int raw_value = io_->digitalRead(MCP::PORT::GPIOB);
    if (raw_value == -1) {
      LOG::ERROR("OC_SM", "ERROR READING MCP");
      ctx_.inputs.new_data = false;
      return;
    } else {
      pinStatus = static_cast<uint8_t>(raw_value);
      if (check_Start() != (pinStatus >> 1) & 0x01) {
        ctx_.inputs.ui.start = (pinStatus >> 1) & 0x01;
        ctx_.inputs.new_data = true;
      }
      if (ctx_.inputs.ui.stop != (pinStatus >> 2) & 0x01) {
        ctx_.inputs.ui.stop = (pinStatus >> 2) & 0x01;
        ctx_.inputs.new_data = true;
      }
      if (ctx_.inputs.ui.pick = (pinStatus >> 3) & 0x01) {
        ctx_.inputs.ui.pick = (pinStatus >> 3) & 0x01;
        ctx_.inputs.new_data = true;
      }
      if (ctx_.inputs.ui.manual_mode != (pinStatus >> 4) & 0x01) {
        ctx_.inputs.ui.manual_mode = (pinStatus >> 4) & 0x01;
        ctx_.inputs.new_data = true;
      }
    }
    if (ctx_.inputs.new_data) {
      if (ctx_.inputs.ui.start) {
        LOG::DEBUG("OC_SM", "Start");
      } else if (ctx_.inputs.ui.stop) {
        LOG::DEBUG("OC_SM", "stop");
      } else if (ctx_.inputs.ui.pick) {
        LOG::DEBUG("OC_SM", "Pick");
      } else if (ctx_.inputs.ui.manual_mode) {
        LOG::DEBUG("OC_SM", "MANUAL MODE");
      }
      ctx_.inputs.new_data = false;
      // ctx_.inputs = Context::Inputs{};
    }
  }
  bool check_Start() { return ctx_.inputs.ui.start && !ctx_.inputs.ui.stop; }
};

} // namespace SM
