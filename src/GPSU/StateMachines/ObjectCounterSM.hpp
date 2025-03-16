// ObjectCounter.hpp
#pragma once
#include "Arduino.h"
#include "StateMachines/StateDefines.hpp"
#include "StateMachines/StateMachine.hpp"
#include "Utility/gpsuUtility.hpp"
#include "string.h"

namespace ObjectCounter {

struct Context {
  using Config = ObjectCounter::Config;
  using Data = ObjectCounter::Data;
  using Inputs = ObjectCounter::Inputs;
  using Event = ObjectCounter::Event;
  using Mode = ObjectCounter::Mode;
  using State = ObjectCounter::State;

  State previous_state;
  Config config;
  Data data;
  Inputs inputs;
  Event event;
  Mode mode = Mode::AUTO;
  Object objects[Config::max_objects];
  uint8_t object_count = 0; // Number of active objects
  uint8_t id_counter = 0;
  // Assignment operator
  Context &operator=(const Context &other) {
    if (this != &other) {
      previous_state = other.previous_state;
      config = other.config;
      data = other.data;
      inputs = other.inputs;
      event = other.event;
      mode = other.mode;
      for (uint8_t i = 0; i < Config::max_objects; i++)
        objects[i] = other.objects[i];
      object_count = other.object_count;
    }
    return *this;
  }
  void addObject(uint32_t current_conveyor_runtime) {
    if (object_count >= Config::max_objects)
      return;
    Object obj;
    obj.id = id_counter;
    obj.state = Objects::State::PLACED;
    obj.placed_in_conveyor = true;
    obj.data = {current_conveyor_runtime,
                current_conveyor_runtime + config.sensor_trigger_delay_ms,
                current_conveyor_runtime + config.picker_arrival_delay_ms, 0,
                0};
    Serial.printf("Object id= %d  is Placed \n", obj.id);
    objects[object_count++] = obj;
    id_counter += 1;
  }

  void updateObjects(uint32_t current_conveyor_runtime) {
    for (uint8_t i = 0; i < object_count; i++) {
      // Transition from PLACED to SENSED
      if (!objects[i].triggered_sensor &&
          current_conveyor_runtime >= objects[i].data.sensor_trigger_time_ms) {
        objects[i].state = Objects::State::SENSED;
        event = Event::SENSOR_TRIGGERED;
        inputs.sensors.photoeye_active = true;
        objects[i].triggered_sensor = true;
        Serial.printf("Object id= %d  is triggered \n", objects[i].id);
        data.production.total_objects_detected++;
      }
      // Transition from SENSED to AT_PICKER and start pick processing
      if (!objects[i].reached_picker &&
          current_conveyor_runtime >= objects[i].data.picker_arrival_time_ms) {
        objects[i].state = Objects::State::AT_PICKER;
        objects[i].reached_picker = true;
        event = Event::AT_PICKER; // Set event
        Serial.printf("Object id= %d  is at Picker \n", objects[i].id);
        startPickerProcessing(objects[i], current_conveyor_runtime);
      }
    }
  }
  void startPickerProcessing(Object &obj, uint32_t current_conveyor_runtime) {
    obj.data.pick_attempt_time_ms =
        current_conveyor_runtime +
        (mode == Mode::AUTO ? config.simulated_pick_delay_ms : 0);

    obj.data.pick_deadline_time_ms =
        current_conveyor_runtime + (mode == Mode::AUTO
                                        ? config.auto_picking_time_limit_ms
                                        : config.manual_picking_time_limit_ms);
  }

  void removeCompletedObjects() {
    uint8_t newCount = 0;
    for (uint8_t i = 0; i < object_count; ++i) {
      if (objects[i].state == Objects::State::PICKED ||
          objects[i].state == Objects::State::FAILED) {
        Serial.printf("Object id= %d  is removed from active list\n",
                      objects[i].id);
      } else {
        objects[newCount++] = objects[i];
        Serial.printf("Object id= %d  is kept in active list\n", objects[i].id);
      }
    }
    object_count = newCount;
  }
};

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

// static inline bool ruuningToOP(const Context &ctx) {
//     return ctx.inputs.timer.current_placement_time_ms >=
//            ctx.config.per_object_time_msec;
//   }
//   static inline bool arrivedOnSensePoint(const Context &ctx) {
//     // will trigger a sensor relay
//     return ctx.inputs.timer.current_placed_object_ms >=
//            ctx.config.object_arrival_duration_sp_ms;
//   }

//   static inline bool objectPlacedToObjectSensed(const Context &ctx) {
//     return auto_mode(ctx) && arrivedOnSensePoint(ctx) ||
//            !auto_mode(ctx) && ctx.inputs.new_input &&
//                ctx.inputs.acknowledge_sensor; // Simulated sensor trigger
//                will
//                                               // sensed by users plc
//   }
//   static inline bool arrivedOnPickPoint(const Context &ctx) {
//     // will trigger a sensor relay
//     return ctx.inputs.timer.current_placed_object_ms >=
//            ctx.config.object_arrival_duration_pp_ms;
//   }
//   static inline bool objectSensedToPicking(const Context &ctx) {
//     return auto_mode(ctx) && arrivedOnPickPoint(ctx) ||
//            !auto_mode(ctx) && arrivedOnPickPoint(ctx) &&
//                ctx.inputs.new_input &&
//                ctx.inputs.pick_arrived; // user plc input to confirm user
//                                         // knows its picking
//   }
//   static inline bool pickingToPick(const Context &ctx) {
//     // will trigger a pick  relay
//     return (auto_mode(ctx) && ctx.inputs.timer.current_placed_object_ms >=
//                                   ctx.config.auto_picking_time_ms) ||
//            (!auto_mode(ctx) && ctx.inputs.new_input && ctx.inputs.pick_now;)
//   }
//   static inline bool pickingToPickFailed(const Context &ctx) {
//     // will trigger alarm relay
//     return (auto_mode(ctx) && false) ||
//            (!auto_mode(ctx) && ctx.inputs.timer.current_placed_object_ms >=
//                                    ctx.config.picking_time_limit_ms;)
//   }

// void updateObjects(uint32_t current_time_ms) {
//     for (uint8_t i = 0; i < object_count; i++) {
//       // Transition from PLACED to SENSED
//       if (!objects[i].sensorTriggered &&
//           current_time_ms >= objects[i].data.sensor_trigger_time_ms) {
//         objects[i].state = Objects::State::SENSED;
//         objects[i].sensorTriggered = true;
//         data.production.total_objects_detected++;
//       }
//       // Transition from SENSED to AT_PICKER and start pick processing
//       if (!objects[i].pickerProcessed &&
//           current_time_ms >= objects[i].data.picker_arrival_time_ms) {
//         objects[i].state = Objects::State::AT_PICKER;
//         objects[i].pickerProcessed = true;
//         startPickerProcessing(objects[i], current_time_ms);
//       }
//     }
//   }

//   void startPickerProcessing(Object &obj, uint32_t current_time_ms) {
//     obj.data.pick_attempt_time_ms = current_time_ms;
//     if (mode == Mode::AUTO)
//       obj.data.pick_deadline_time_ms =
//           current_time_ms + config.auto_picking_time_ms;
//     else
//       obj.data.pick_deadline_time_ms =
//           current_time_ms + config.picking_time_limit_ms;
//   }