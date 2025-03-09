// #ifndef WATER_LEVEL_SM_HPP
// #define WATER_LEVEL_SM_HPP

// #include "Arduino.h"
// #include "StateMachines/StateMachine.hpp"

// #define WL_STATE(state) static_cast<uint8_t>(SM::StateWaterLevel::state)

// namespace SM {
// class WaterLevelSM; // Forward declaration
// } // namespace SM

// namespace WL_SM {

// // **Condition Functions**
// // Overflow is detected if the current water level exceeds the tank capacity
// inline bool detectedOverflow(const SM::WaterLevelInputs &data,
//                              const SM::WaterLevelConfig &config) {
//   return data.currentLevel > config.tankMaxCapacityLitre;
// }
// // Once overflow conditions subside (e.g., water level falls back to safe
// // limits)
// inline bool overflowResolved(const SM::WaterLevelInputs &data,
//                              const SM::WaterLevelConfig &config) {
//   return data.currentLevel <= config.tankMaxCapacityLitre;
// }
// // Check if the start filling command is issued
// inline bool waterDetected(const SM::WaterLevelInputs &data,
//                           const SM::WaterLevelConfig &config) {
//   // water is detected and filling should start.
//   return data.sensorADC > config.sensorMinSensivityLitre;
// }
// inline bool startFillingCondition(const SM::WaterLevelInputs &data,
//                                   const SM::WaterLevelConfig &config) {
//   return data.start_filling_flag;
// }

// inline bool reachedPartial(const SM::WaterLevelInputs &data,
//                            const SM::WaterLevelConfig &config) {
//   // Transition when the water level is moderately high (e.g., > 500 units)
//   return data.currentLevel >= config.partialLevelLow &&
//          data.currentLevel < config.partialLevelHigh;
// }

// inline bool reachedFull(const SM::WaterLevelInputs &data,
//                         const SM::WaterLevelConfig &config) {
//   // Transition when water level nears the tank capacity (e.g., between 1900
//   and
//   // 2000 units)
//   return data.currentLevel >= config.partialLevelHigh &&
//          data.currentLevel <= config.tankCapacityLitre;
// }
// inline bool droppedBelowFull(const SM::WaterLevelInputs &data,
//                              const SM::WaterLevelConfig &config) {
//   // Transition from FULL to draining when the level falls below a threshold
//   return data.currentLevel < config.drainingMark;
// }
// inline bool nearlyEmpty(const SM::WaterLevelInputs &data,
//                         const SM::WaterLevelConfig &config) {
//   // Consider the tank empty if current level is almost zero
//   return data.currentLevel <= config.sensorMinSensivityLitre;
// }

// // Check if the start draining command is issued
// inline bool startDrainingCondition(const SM::WaterLevelInputs &data,
//                                    const SM::WaterLevelConfig &config) {
//   return data.start_draining_flag;
// }

// // Check if the stop command is issued
// inline bool stopCondition(const SM::WaterLevelInputs &data,
//                           const SM::WaterLevelConfig &config) {
//   return data.stop_flag;
// }

// // Check if the tank is full
// inline bool levelFullCondition(
//     const SM::WaterLevelInputs &data.const SM::WaterLevelConfig &config) {
//   return data.currentLevel >= config.tankCapacityLitre;
// }
// // Check if the tank is empty
// inline bool levelEmptyCondition(const SM::WaterLevelInputs &data,
//                                 const SM::WaterLevelConfig &config) {
//   return data.currentLevel <= 0.0f;
// }
// inline bool partialLevelCondition(const SM::WaterLevelInputs &data,
//                                   const SM::WaterLevelConfig &config) {
//   return data.currentLevel >= data.targetLevel +
//   config.sensorMinSensivityLitre;
// }
// // Activate draining (could mean reversing the pump or opening a drain valve)
// inline WLCommand actionStartDraining(const SM::WaterLevelInputs &data,
//                                      const SM::WaterLevelConfig &config) {
//   WLCommand cmd;
//   cmd.command = SM::CommandsWL::START_DRAIN;
//   cmd.data.pumpSpeed = config.pumpDrainFlowRate;
//   cmd.data.open_drain_valve = true;
//   return cmd;
// }
// inline WLCommand actionDrained(const SM::WaterLevelInputs &data,
//                                const SM::WaterLevelConfig &config) {
//   // When drained, turn off the pump.
//   WLCommand cmd;
//   cmd.command = SM::CommandsWL::STOP;
//   cmd.data.pumpSpeed = 0;
//   cmd.data.open_drain_valve = false;
//   return cmd;
// }
// inline WLCommand actionHandleOverflow(const SM::WaterLevelInputs &data,
//                                       const SM::WaterLevelConfig &config) {
//   // Stop filling or trigger an alarm in case of overflow.
//   WLCommand cmd;
//   cmd.command = SM::CommandsWL::TRIGGER_ALARM;
//   cmd.data.pumpSpeed = 0;
//   cmd.data.alarm = true;
//   return cmd;
// }

// // **Action Functions**
// inline WLCommand actionEmptyToFilling(const SM::WaterLevelInputs &data,
//                                       const SM::WaterLevelConfig &config) {
//   WLCommand cmd;
//   cmd.command = SM::CommandsWL::START_FILL;
//   cmd.data.pumpSpeed = config.pumpFillFlowRate; // Start filling
//   return cmd;
// }

// inline WLCommand actionStop(const SM::WaterLevelInputs &data,
//                             const SM::WaterLevelConfig &config) {
//   WLCommand cmd;
//   cmd.command = SM::CommandsWL::STOP;
//   cmd.data.pumpSpeed = 0;
//   return cmd;
// }

// inline WLCommand actionFullToDraining(const SM::WaterLevelInputs &data,
//                                       const SM::WaterLevelConfig &config) {
//   WLCommand cmd;
//   cmd.command = SM::CommandsWL::START_DRAIN;
//   cmd.data.pumpSpeed = config_.pumpDrainFlowRate;
//   cmd.data.open_drain_valve = true;
//   return cmd;
// }

// inline WLCommand actionPartialToFilling(const SM::WaterLevelInputs &data,
//                                         const SM::WaterLevelConfig &config) {
//   WLCommand cmd;
//   cmd.command = SM::CommandsWL::START_DRAIN;
//   cmd.data.pumpSpeed = config.pumpFillFlowRate;
//   cmd.data.targetLevel = data.targetLevel;
//   return cmd;
// }

// inline WLCommand actionPartialToDraining(const SM::WaterLevelInputs &data,
//                                          const SM::WaterLevelConfig &config)
//                                          {
//   WLCommand cmd;
//   cmd.command = SM::CommandsWL::START_DRAIN;
//   cmd.data.pumpSpeed = config.pumpDrainFlowRate;
//   cmd.data.targetLevel = data.targetLevel;
//   cmd.data.open_drain_valve = true;
//   return cmd;
// }

// // **Transition Table**
// constexpr SM::Transition<SM::WaterLevelData> WaterLevelTransitions[] = {

//     {WL_STATE(EMPTY), WL_STATE(OVERFLOW), detectedOverflow,
//      actionHandleOverflow},
//     {WL_STATE(FILLING), WL_STATE(OVERFLOW), detectedOverflow,
//      actionHandleOverflow},
//     {WL_STATE(PARTIAL_FILLED), WL_STATE(OVERFLOW), detectedOverflow,
//      actionHandleOverflow},
//     {WL_STATE(FULL), WL_STATE(OVERFLOW), detectedOverflow,
//      actionHandleOverflow},
//     {WL_STATE(OVERFLOW), WL_STATE(DRAINING), overflowResolved,
//      actionStartDraining},
//     // From EMPTY to START FILLING when start filling is commanded
//     {WL_STATE(EMPTY), WL_STATE(START_FILLING), startFillingCondition,
//      actionEmptyToFilling},
//     // From START FILLING  to FILLING when water detected
//     {WL_STATE(START_FILLING), WL_STATE(FILLING), waterDetected,
//      actionEmptyToFilling},
//     // From FILLING to PARTIAL_FILLED when stopped before full
//     {WL_STATE(FILLING), WL_STATE(PARTIAL_FILLED), stopCondition, actionStop},
//     // From FILLING to PARTIAL_FILLED when stopped before full
//     {WL_STATE(FILLING), WL_STATE(PARTIAL_FILLED), partialLevelCondition,
//      actionStop},
//     // From PARTIAL_FILLED to FULL when the level reaches capacity
//     {WL_STATE(PARTIAL_FILLED), WL_STATE(FULL), reachedFull, actionStop},
//     // From FILLING to FULL when the level reaches capacity
//     {WL_STATE(FILLING), WL_STATE(FULL), levelFullCondition, actionStop},

//     // From FULL to DRAINING when start draining is commanded
//     {WL_STATE(FULL), WL_STATE(DRAINING), startDrainingCondition,
//      actionFullToDraining},
//     // From DRAINING to EMPTY when the level reaches 0
//     {WL_STATE(DRAINING), WL_STATE(EMPTY), levelEmptyCondition, actionStop},
//     // From DRAINING to PARTIAL_FILLED when stopped before empty
//     {WL_STATE(DRAINING), WL_STATE(PARTIAL_FILLED), stopCondition,
//     actionStop},
//     // From DRAINING to PARTIAL_FILLED when target level reached
//     {WL_STATE(DRAINING), WL_STATE(PARTIAL_FILLED), partialLevelCondition,
//      actionStop},
//     // From PARTIAL_FILLED to FILLING when start filling is commanded
//     {WL_STATE(PARTIAL_FILLED), WL_STATE(FILLING), startFillingCondition,
//      actionPartialToFilling},
//     // From PARTIAL_FILLED to DRAINING when start draining is commanded
//     {WL_STATE(PARTIAL_FILLED), WL_STATE(DRAINING), startDrainingCondition,
//      actionPartialToDraining},
// };

// } // namespace WL_SM

// namespace SM {

// class WaterLevelSM
//     : public StateMachine<WaterLevelSM, StateWaterLevel, WaterLevelConfig,
//                           WaterLevelInputs, WLCommand, CommandsWL,
//                           WL_SM::WaterLevelTransitions,
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
//                      WaterLevelInputs) {}

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