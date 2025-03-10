// WaterLevel.hpp
#pragma once
#include "StateMachine.hpp"

namespace WaterLevel {

enum class State : uint8_t {
  EMPTY = 0,
  START_FILLING,
  FILLING,
  DRAINING,
  PARTIAL_FILLED,
  FULL,
  OVERFLOW,
};
enum class CommandType {
  NONE,
  LEVEL_UPDATE,
  START_FILL,
  FILL_TO_LEVEL,
  STOP,
  START_DRAIN,
  TRIGGER_ALARM,
};
struct CommandData {
  int pump_speed;
  float target_level;
  bool open_drain_valve;
  bool open_fill_valve;
  bool alarm;
};
struct Command {
  CommandType command;
  CommandData data;
};
struct Config {
  float alarm_level = 2100;
  float max_capacity = 2000;
  float partial_mark = 500;
  float draining_mark = 1800;
  int fill_rate = 100;
  int drain_rate = -50;
  float sensor_min_sensitivity = 10;
  float hysteresis = 15;
};

struct Inputs {
  struct Sensors {
    float raw_adc_value;
    float measured_level;
    bool drain_valve_state = false;
    bool fill_valve_state = false;
  } sensors;
  struct UserCommand {
    float new_target_level;
    bool fill_request;
    bool drain_request;
    bool stop;
  } user_command;
};
struct Data {
  float current_level;
  float current_target_level;
};
struct Context {
  using Config = WaterLevel::Config;
  using Data = WaterLevel::Data;
  using Inputs = WaterLevel::Inputs;
  State previous_state;
  Config config;
  Data data;
  Inputs inputs;
};

struct Traits {
  using Context = WaterLevel::Context;
  using Config = WaterLevel::Config;
  using Data = WaterLevel::Data;
  using Inputs = WaterLevel::Inputs;
  using State = WaterLevel::State;
  using Command = WaterLevel::Command;

  struct Transition {
    State from;
    State to;
    bool (*condition)(const Context &);
    Command (*action)(const Context &);
  };

  static bool detectedOverflow(const Context &ctx) {
    return ctx.data.current_level > ctx.config.max_capacity;
  }
  static bool overflowResolved(const Context &ctx) {
    return ctx.data.current_level <= ctx.config.max_capacity;
  }

  static bool waterDetected(const Context &ctx) {
    // water is detected and filling should start.
    return ctx.inputs.sensors.raw_adc_value > ctx.config.sensor_min_sensitivity;
  }
  static bool startFillingCondition(const Context &ctx) {
    return ctx.inputs.user_command.fill_request;
  }
  static inline bool reachedEmpty(const Context &ctx) {
    return ctx.inputs.sensors.measured_level <= 0.0f + ctx.config.hysteresis;
  }
  static bool reachedPartial(const Context &ctx) {
    // Transition when the water level is moderately high (e.g., > 500 units)
    return ctx.data.current_level >= ctx.config.partial_mark;
  }
  static bool reachedLevel(const Context &ctx) {
    // Transition when the water level reached a particular value
    return ctx.data.current_level + ctx.config.hysteresis >=
           ctx.data.current_target_level;
  }
  static bool reachedFull(const Context &ctx) {

    return ctx.data.current_level + ctx.config.hysteresis <=
           ctx.config.max_capacity;
  }
  static bool droppedBelowFull(const Context &ctx) {
    // Transition from FULL to draining when the level falls below a threshold
    return ctx.data.current_level < ctx.config.draining_mark;
  }
  static bool startDrainingCondition(const Context &ctx) {
    return ctx.inputs.user_command.drain_request;
  }
  static bool stopCondition(const Context &ctx) {
    return ctx.inputs.user_command.stop;
  }
  static Command actionStartDraining(const Context &ctx) {
    Command cmd;
    cmd.command = CommandType::START_DRAIN;
    cmd.data.pump_speed = ctx.config.drain_rate;
    cmd.data.open_drain_valve = true;
    return cmd;
  }
  static Command actionHandleOverflow(const Context &ctx) {
    // Stop filling or trigger an alarm in case of overflow.
    Command cmd;
    cmd.command = CommandType::TRIGGER_ALARM;
    cmd.data.pump_speed = 0;
    cmd.data.open_fill_valve = false;
    cmd.data.alarm = true;
    return cmd;
  }
  static Command actionEmptyToFilling(const Context &ctx) {
    Command cmd;
    cmd.command = CommandType::START_FILL;
    cmd.data.pump_speed = ctx.config.fill_rate; // Start filling
    cmd.data.open_fill_valve = true;
    return cmd;
  }
  static Command actionStop(const Context &ctx) {
    Command cmd;
    cmd.command = CommandType::STOP;
    cmd.data.pump_speed = 0;
    cmd.data.open_fill_valve = false;
    cmd.data.open_drain_valve = false;
    return cmd;
  }
  static Command actionFullToDraining(const Context &ctx) {
    Command cmd;
    cmd.command = CommandType::START_DRAIN;
    cmd.data.pump_speed = ctx.config.drain_rate;
    cmd.data.open_drain_valve = true;
    return cmd;
  }

  static Command actionPartialToFilling(const Context &ctx) {
    Command cmd;
    cmd.command = CommandType::START_FILL;
    cmd.data.pump_speed = ctx.config.fill_rate;
    cmd.data.target_level = ctx.inputs.user_command.new_target_level;
    cmd.data.open_fill_valve = true;
    return cmd;
  }

  static Command actionPartialToDraining(const Context &ctx) {
    Command cmd;
    cmd.command = CommandType::START_DRAIN;
    cmd.data.pump_speed = ctx.config.drain_rate;
    cmd.data.target_level = ctx.inputs.user_command.new_target_level;
    cmd.data.open_drain_valve = true;
    return cmd;
  }

  static constexpr std::array<Transition, 17> transitions = {{
      // Transitions to OVERFLOW
      {State::EMPTY, State::OVERFLOW, detectedOverflow, actionHandleOverflow},
      {State::FILLING, State::OVERFLOW, detectedOverflow, actionHandleOverflow},
      {State::PARTIAL_FILLED, State::OVERFLOW, detectedOverflow,
       actionHandleOverflow},
      {State::FULL, State::OVERFLOW, detectedOverflow, actionHandleOverflow},

      // Overflow resolution and draining
      {State::OVERFLOW, State::DRAINING, overflowResolved, actionStartDraining},

      // Filling transitions
      {State::EMPTY, State::START_FILLING, startFillingCondition,
       actionEmptyToFilling},
      {State::START_FILLING, State::FILLING, waterDetected,
       actionEmptyToFilling},
      {State::FILLING, State::PARTIAL_FILLED, stopCondition, actionStop},
      {State::FILLING, State::PARTIAL_FILLED, reachedPartial, actionStop},
      {State::PARTIAL_FILLED, State::FULL, reachedFull, actionStop},
      {State::FILLING, State::FULL, reachedFull, actionStop},

      // Draining transitions
      {State::FULL, State::DRAINING, startDrainingCondition,
       actionFullToDraining},
      {State::DRAINING, State::EMPTY, reachedEmpty, actionStop},
      {State::DRAINING, State::PARTIAL_FILLED, stopCondition, actionStop},
      {State::DRAINING, State::PARTIAL_FILLED, reachedPartial, actionStop},

      // Partial filled state transitions
      {State::PARTIAL_FILLED, State::FILLING, startFillingCondition,
       actionPartialToFilling},
      {State::PARTIAL_FILLED, State::DRAINING, startDrainingCondition,
       actionPartialToDraining},
  }};
};

}; // namespace WaterLevel
namespace SM {
class WaterLevelSM : public StateMachine<WaterLevel::Traits, 17> {

public:
  using Traits = WaterLevel::Traits;
  using Context = Traits::Context;
  using State = Traits::State;
  using Inputs = WaterLevel::Inputs;
  // Constructor with initial data
  WaterLevelSM(const Context &context, State state)
      : StateMachine(context, state) {}
  WaterLevelSM(const Context &context) : WaterLevelSM(context, State::EMPTY) {}
  WaterLevelSM() : WaterLevelSM(Context{}, State::EMPTY) {}

  void updateInternalState(const Inputs &input) override { // Correct override
    ctx_.data.current_level = input.sensors.measured_level;
    ctx_.data.current_target_level = input.user_command.new_target_level;
  }
  // Get the current state as a string for debugging or display
  String getStateString() const {
    switch (current()) {
    case State::EMPTY:
      return "EMPTY";
    case State::START_FILLING:
      return "START_FILLING";
    case State::FILLING:
      return "FILLING";
    case State::DRAINING:
      return "DRAINING";
    case State::PARTIAL_FILLED:
      return "PARTIAL_FILLED";
    case State::FULL:
      return "FULL";
    case State::OVERFLOW:
      return "OVERFLOW";
    default:
      return "UNKNOWN";
    }
  }
};

} // namespace SM

// #include "Arduino.h"
// #include "StateMachines/StateMachine.hpp"

// #define WL_STATE(state) static_cast<uint8_t>(SM::StateWaterLevel::state)

// namespace SM {
// class WaterLevelSM; // Forward declaration
// } // namespace SM

// namespace WL_SM {

// // **Condition Functions**
// // Overflow is detected if the current water level exceeds the tank capacity
// inline bool detectedOverflow(const SM::StateWaterLevel oldstate,
//                              const SM::WaterLevelData &data,
//                              const SM::WaterLevelInputs &input,
//                              const SM::WaterLevelConfig &config) {
//   return data.currentLevel > config.tankMaxCapacityLitre;
// }
// // Once overflow conditions subside (e.g., water level falls back to safe
// // limits)
// inline bool overflowResolved(const SM::StateWaterLevel oldstate,
//                              const SM::WaterLevelData &data,
//                              const SM::WaterLevelInputs &input,
//                              const SM::WaterLevelConfig &config) {
//   return data.currentLevel <= config.tankMaxCapacityLitre;
// }
// // Check if the start filling command is issued
// inline bool waterDetected(const SM::StateWaterLevel oldstate,
//                           const SM::WaterLevelData &data,
//                           const SM::WaterLevelInputs &input,
//                           const SM::WaterLevelConfig &config) {
//   // water is detected and filling should start.
//   return input.sensorADC > config.sensorMinSensivityLitre;
// }
// inline bool startFillingCondition(const SM::StateWaterLevel oldstate,
//                                   const SM::WaterLevelData &data,
//                                   const SM::WaterLevelInputs &input,
//                                   const SM::WaterLevelConfig &config) {
//   return input.start_filling_flag;
// }

// inline bool reachedPartial(const SM::StateWaterLevel oldstate,
//                            const SM::WaterLevelData &data,
//                            const SM::WaterLevelInputs &input,
//                            const SM::WaterLevelConfig &config) {
//   // Transition when the water level is moderately high (e.g., > 500 units)
//   return data.currentLevel >= config.partialLevelLow &&
//          data.currentLevel < config.partialLevelHigh;
// }

// inline bool reachedFull(const SM::StateWaterLevel oldstate,
//                         const SM::WaterLevelData &data,
//                         const SM::WaterLevelInputs &input,
//                         const SM::WaterLevelConfig &config) {
//   // Transition when water level nears the tank capacity (e.g., between 1900
//   and
//   // 2000 units)
//   return data.currentLevel >= config.partialLevelHigh &&
//          data.currentLevel <= config.tankCapacityLitre;
// }
// inline bool droppedBelowFull(const SM::StateWaterLevel oldstate,
//                              const SM::WaterLevelData &data,
//                              const SM::WaterLevelInputs &input,
//                              const SM::WaterLevelConfig &config) {
//   // Transition from FULL to draining when the level falls below a threshold
//   return data.currentLevel < config.drainingMark;
// }
// inline bool nearlyEmpty(const SM::StateWaterLevel oldstate,
//                         const SM::WaterLevelData &data,
//                         const SM::WaterLevelInputs &input,
//                         const SM::WaterLevelConfig &config) {
//   // Consider the tank empty if current level is almost zero
//   return data.currentLevel <= config.sensorMinSensivityLitre;
// }

// // Check if the start draining command is issued
// inline bool startDrainingCondition(const SM::StateWaterLevel oldstate,
//                                    const SM::WaterLevelData &data,
//                                    const SM::WaterLevelInputs &input,
//                                    const SM::WaterLevelConfig &config) {
//   return input.start_draining_flag;
// }

// // Check if the stop command is issued
// inline bool stopCondition(const SM::StateWaterLevel oldstate,
//                           const SM::WaterLevelData &data,
//                           const SM::WaterLevelInputs &input,
//                           const SM::WaterLevelConfig &config) {
//   return input.stop_flag;
// }

// // Check if the tank is full
// inline bool levelFullCondition(const SM::StateWaterLevel oldstate,
//                                const SM::WaterLevelData &data,
//                                const SM::WaterLevelInputs &input,
//                                const SM::WaterLevelConfig &config) {
//   return data.currentLevel >= config.tankCapacityLitre;
// }
// // Check if the tank is empty
// inline bool levelEmptyCondition(const SM::StateWaterLevel oldstate,
//                                 const SM::WaterLevelData &data,
//                                 const SM::WaterLevelInputs &input,
//                                 const SM::WaterLevelConfig &config) {
//   return data.currentLevel <= 0.0f;
// }
// inline bool partialLevelCondition(const SM::StateWaterLevel oldstate,
//                                   const SM::WaterLevelData &data,
//                                   const SM::WaterLevelInputs &input,
//                                   const SM::WaterLevelConfig &config) {
//   return data.currentLevel >=
//          input.targetLevel + config.sensorMinSensivityLitre;
// }
// // Activate draining (could mean reversing the pump or opening a drain valve)
// inline SM::WLCommand actionStartDraining(const SM::WaterLevelInputs &data,
//                                          const SM::WaterLevelConfig &config)
//                                          {
//   SM::WLCommand cmd;
//   cmd.command = SM::CommandsWL::START_DRAIN;
//   cmd.data.pumpSpeed = config.pumpDrainFlowRate;
//   cmd.data.open_drain_valve = true;
//   return cmd;
// }
// inline SM::WLCommand actionDrained(const SM::WaterLevelInputs &data,
//                                    const SM::WaterLevelConfig &config) {
//   // When drained, turn off the pump.
//   SM::WLCommand cmd;
//   cmd.command = SM::CommandsWL::STOP;
//   cmd.data.pumpSpeed = 0;
//   cmd.data.open_drain_valve = false;
//   return cmd;
// }
// inline SM::WLCommand actionHandleOverflow(const SM::WaterLevelInputs &data,
//                                           const SM::WaterLevelConfig &config)
//                                           {
//   // Stop filling or trigger an alarm in case of overflow.
//   SM::WLCommand cmd;
//   cmd.command = SM::CommandsWL::TRIGGER_ALARM;
//   cmd.data.pumpSpeed = 0;
//   cmd.data.alarm = true;
//   return cmd;
// }

// // **Action Functions**
// inline SM::WLCommand actionEmptyToFilling(const SM::WaterLevelInputs &data,
//                                           const SM::WaterLevelConfig &config)
//                                           {
//   SM::WLCommand cmd;
//   cmd.command = SM::CommandsWL::START_FILL;
//   cmd.data.pumpSpeed = config.pumpFillFlowRate; // Start filling
//   return cmd;
// }

// inline SM::WLCommand actionStop(const SM::WaterLevelInputs &data,
//                                 const SM::WaterLevelConfig &config) {
//   SM::WLCommand cmd;
//   cmd.command = SM::CommandsWL::STOP;
//   cmd.data.pumpSpeed = 0;
//   return cmd;
// }

// inline SM::WLCommand actionFullToDraining(const SM::WaterLevelInputs &data,
//                                           const SM::WaterLevelConfig &config)
//                                           {
//   SM::WLCommand cmd;
//   cmd.command = SM::CommandsWL::START_DRAIN;
//   cmd.data.pumpSpeed = config.pumpDrainFlowRate;
//   cmd.data.open_drain_valve = true;
//   return cmd;
// }

// inline SM::WLCommand
// actionPartialToFilling(const SM::WaterLevelInputs &data,
//                        const SM::WaterLevelConfig &config) {
//   SM::WLCommand cmd;
//   cmd.command = SM::CommandsWL::START_DRAIN;
//   cmd.data.pumpSpeed = config.pumpFillFlowRate;
//   cmd.data.targetLevel = data.targetLevel;
//   return cmd;
// }

// inline SM::WLCommand
// actionPartialToDraining(const SM::WaterLevelInputs &data,
//                         const SM::WaterLevelConfig &config) {
//   SM::WLCommand cmd;
//   cmd.command = SM::CommandsWL::START_DRAIN;
//   cmd.data.pumpSpeed = config.pumpDrainFlowRate;
//   cmd.data.targetLevel = data.targetLevel;
//   cmd.data.open_drain_valve = true;
//   return cmd;
// }

// // **Transition Table**
// constexpr SM::Transition<SM::StateWaterLevel, SM::WaterLevelConfig,
//                          SM::WaterLevelData, SM::WaterLevelInputs,
//                          SM::WLCommand>
//     WaterLevelTransitions[] = {

//         {WL_STATE(EMPTY), WL_STATE(OVERFLOW), detectedOverflow,
//          actionHandleOverflow},
//         {WL_STATE(FILLING), WL_STATE(OVERFLOW), detectedOverflow,
//          actionHandleOverflow},
//         {WL_STATE(PARTIAL_FILLED), WL_STATE(OVERFLOW), detectedOverflow,
//          actionHandleOverflow},
//         {WL_STATE(FULL), WL_STATE(OVERFLOW), detectedOverflow,
//          actionHandleOverflow},
//         {WL_STATE(OVERFLOW), WL_STATE(DRAINING), overflowResolved,
//          actionStartDraining},
//         // From EMPTY to START FILLING when start filling is commanded
//         {WL_STATE(EMPTY), WL_STATE(START_FILLING), startFillingCondition,
//          actionEmptyToFilling},
//         // From START FILLING  to FILLING when water detected
//         {WL_STATE(START_FILLING), WL_STATE(FILLING), waterDetected,
//          actionEmptyToFilling},
//         // From FILLING to PARTIAL_FILLED when stopped before full
//         {WL_STATE(FILLING), WL_STATE(PARTIAL_FILLED), stopCondition,
//          actionStop},
//         // From FILLING to PARTIAL_FILLED when stopped before full
//         {WL_STATE(FILLING), WL_STATE(PARTIAL_FILLED), partialLevelCondition,
//          actionStop},
//         // From PARTIAL_FILLED to FULL when the level reaches capacity
//         {WL_STATE(PARTIAL_FILLED), WL_STATE(FULL), reachedFull, actionStop},
//         // From FILLING to FULL when the level reaches capacity
//         {WL_STATE(FILLING), WL_STATE(FULL), levelFullCondition, actionStop},

//         // From FULL to DRAINING when start draining is commanded
//         {WL_STATE(FULL), WL_STATE(DRAINING), startDrainingCondition,
//          actionFullToDraining},
//         // From DRAINING to EMPTY when the level reaches 0
//         {WL_STATE(DRAINING), WL_STATE(EMPTY), levelEmptyCondition,
//         actionStop},
//         // From DRAINING to PARTIAL_FILLED when stopped before empty
//         {WL_STATE(DRAINING), WL_STATE(PARTIAL_FILLED), stopCondition,
//          actionStop},
//         // From DRAINING to PARTIAL_FILLED when target level reached
//         {WL_STATE(DRAINING), WL_STATE(PARTIAL_FILLED), partialLevelCondition,
//          actionStop},
//         // From PARTIAL_FILLED to FILLING when start filling is commanded
//         {WL_STATE(PARTIAL_FILLED), WL_STATE(FILLING), startFillingCondition,
//          actionPartialToFilling},
//         // From PARTIAL_FILLED to DRAINING when start draining is commanded
//         {WL_STATE(PARTIAL_FILLED), WL_STATE(DRAINING),
//         startDrainingCondition,
//          actionPartialToDraining},
// };

// } // namespace WL_SM

// namespace SM {

// class WaterLevelSM
//     : public StateMachine<WaterLevelSM, StateWaterLevel, WaterLevelConfig,
//                           WaterLevelData, WaterLevelInputs, WLCommand,
//                           CommandsWL, WL_SM::WaterLevelTransitions,
//                           sizeof(WL_SM::WaterLevelTransitions) /
//                               sizeof(WL_SM::WaterLevelTransitions[0])> {
// public:
//   // Constructor with initial data
//   WaterLevelSM(const WaterLevelConfig &config, const WaterLevelInputs
//   &initData)
//       : StateMachine(StateWaterLevel::EMPTY, config, initData) {}
//   WaterLevelSM(const WaterLevelConfig &config)
//       : StateMachine(StateWaterLevel::EMPTY, config, WaterLevelInputs{}) {}
//   WaterLevelSM(const WaterLevelInputs &initData)
//       : StateMachine(StateWaterLevel::EMPTY, WaterLevelConfig{}, initData) {}

//   // Default constructor
//   constexpr WaterLevelSM()
//       : StateMachine(StateWaterLevel::EMPTY, WaterLevelConfig{},
//                      WaterLevelInputs{}) {}
//   void updateInternalState(const WaterLevelInputs &input) override {
//     data_.currentLevel = input.sensorADC * config_.sensorVoltperLitre;
//     data_.currentTargetLevel = input.targetLevel;
//   }
//   // Get the current state as a string for debugging or display
//   String getStateString() const {
//     switch (getState()) {
//     case StateWaterLevel::EMPTY:
//       return "EMPTY";
//     case StateWaterLevel::START_FILLING:
//       return "START_FILLING";
//     case StateWaterLevel::FILLING:
//       return "FILLING";
//     case StateWaterLevel::DRAINING:
//       return "DRAINING";
//     case StateWaterLevel::PARTIAL_FILLED:
//       return "PARTIAL_FILLED";
//     case StateWaterLevel::FULL:
//       return "FULL";
//     case StateWaterLevel::OVERFLOW:
//       return "OVERFLOW";
//     default:
//       return "UNKNOWN";
//     }
//   }
// };

// } // namespace SM

// #endif // WATER_LEVEL_SM_HPP