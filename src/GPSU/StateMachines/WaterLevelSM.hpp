#ifndef WATER_LEVEL_SM_HPP
#define WATER_LEVEL_SM_HPP

#include "Arduino.h"
#include "StateMachines/StateMachine.hpp"

#define WL_STATE(state) static_cast<uint8_t>(SM::StateWaterLevel::state)

namespace SM {
class WaterLevelSM; // Forward declaration
} // namespace SM

namespace WL_SM {
// Event emitter (placeholder, can be extended by application)
static void emitEvent(SM::StateWaterLevel state) {}

// **Condition Functions**
// Overflow is detected if the current water level exceeds the tank capacity
bool detectedOverflow(const SM::WaterLevelData &data,
                      const SM::WaterLevelConfig &config) {
  return data.currentLevel > config.tankMaxCapacityLitre;
}
// Once overflow conditions subside (e.g., water level falls back to safe
// limits)
bool overflowResolved(const SM::WaterLevelData &data,
                      const SM::WaterLevelConfig &config) {
  return data.currentLevel <= config.tankMaxCapacityLitre;
}
// Check if the start filling command is issued
bool waterDetected(const SM::WaterLevelData &data,
                   const SM::WaterLevelConfig &config) {
  // water is detected and filling should start.
  return data.sensorADC > config.sensorMinSensivityLitre;
}
bool startFillingCondition(const SM::WaterLevelData &data,
                           const SM::WaterLevelConfig &config) {
  return data.start_filling_flag & waterDetected();
}

bool reachedPartial(const SM::WaterLevelData &data,
                    const SM::WaterLevelConfig &config) {
  // Transition when the water level is moderately high (e.g., > 500 units)
  return data.currentLevel >= config.partialLevelLow &&
         data.currentLevel < config.partialLevelHigh;
}

bool reachedFull(const SM::WaterLevelData &data,
                 const SM::WaterLevelConfig &config) {
  // Transition when water level nears the tank capacity (e.g., between 1900 and
  // 2000 units)
  return data.currentLevel >= config.partialLevelHigh &&
         data.currentLevel <= config.tankCapacityLitre;
}
bool droppedBelowFull(const SM::WaterLevelData &data,
                      const SM::WaterLevelConfig &config) {
  // Transition from FULL to draining when the level falls below a threshold
  return data.currentLevel < config.drainingMark;
}
bool nearlyEmpty(const SM::WaterLevelData &data,
                 const SM::WaterLevelConfig &config) {
  // Consider the tank empty if current level is almost zero
  return data.currentLevel <= config.sensorMinSensivityLitre;
}

// Activate draining (could mean reversing the pump or opening a drain valve)
void actionStartDraining(const SM::WaterLevelData &data,
                         const SM::WaterLevelConfig &config) {

  data.pumpSpeed = config_.pumpDrainFlowRate;
  data.open_drain_valve = true;
}
void actionDrained(const SM::WaterLevelData &data,
                   const SM::WaterLevelConfig &config) {
  // When drained, turn off the pump.
  data.pumpSpeed = 0;
  data.open_drain_valve = false;
}
void actionHandleOverflow(const SM::WaterLevelData &data,
                          const SM::WaterLevelConfig &config) {
  // Stop filling or trigger an alarm in case of overflow.
  stopAction();
  data.alarm = true;
}

// Check if the start draining command is issued
bool startDrainingCondition(const SM::WaterLevelData &data,
                            const SM::WaterLevelConfig &config) {
  return data.start_draining_flag;
}

// Check if the stop command is issued
bool stopCondition(const SM::WaterLevelData &data,
                   const SM::WaterLevelConfig &config) {
  return data.stop_flag;
}

// Check if the tank is full
bool levelFullCondition(
    const SM::WaterLevelData &data.const SM::WaterLevelConfig &config) {
  return data.currentLevel >= config.tankCapacityLitre;
}

// Check if the tank is empty
bool levelEmptyCondition(const, SM::WaterLevelData &data,
                         const SM::WaterLevelConfig &config) {
  return data.currentLevel <= 0.0f;
}

// **Action Functions**
void emptyToFillingAction(SM::WaterLevelData &data,
                          const SM::WaterLevelConfig &config) {
  data.pumpSpeed = config.pumpFillFlowRate; // Start filling
  data.start_filling_flag = false;          // Clear the flag
}

void stopAction(SM::WaterLevelData &data, const SM::WaterLevelConfig &config) {
  data.pumpSpeed = 0; // Stop the pump
}

void stopActionClearFlag(SM::WaterLevelData &data,
                         const SM::WaterLevelConfig &config) {
  data.pumpSpeed = 0;
  data.stop_flag = false; // Clear the stop flag
}

void fullToDrainingAction(SM::WaterLevelData &data,
                          const SM::WaterLevelConfig &config) {
  data.pumpSpeed = config_.pumpDrainFlowRate; // Start draining
  data.start_draining_flag = false;           // Clear the flag
}

void partialToFillingAction(SM::WaterLevelData &data,
                            const SM::WaterLevelConfig &config) {
  data.pumpSpeed = config.pumpFillFlowRate; // Start filling
  data.start_filling_flag = false;          // Clear the flag
}

inline void partialToDrainingAction(SM::WaterLevelData &data,
                                    const SM::WaterLevelConfig &config) {
  data.pumpSpeed = config.pumpDrainFlowRate; // Start draining
  data.start_draining_flag = false;          // Clear the flag
}

// **Transition Table**
constexpr SM::Transition<SM::WaterLevelData> WaterLevelTransitions[] = {

    {WL_STATE(EMPTY), WL_STATE(OVERFLOW), detectedOverflow,
     actionHandleOverflow},
    {WL_STATE(FILLING), WL_STATE(OVERFLOW), detectedOverflow,
     actionHandleOverflow},
    {WL_STATE(PARTIAL_FILLED), WL_STATE(OVERFLOW), detectedOverflow,
     actionHandleOverflow},
    {WL_STATE(FULL), WL_STATE(OVERFLOW), detectedOverflow,
     actionHandleOverflow},
    {WL_STATE(OVERFLOW), WL_STATE(DRAINING), overflowResolved,
     actionStartDraining},

    // From EMPTY to FILLING when start filling is commanded
    {WL_STATE(EMPTY), WL_STATE(FILLING), startFillingCondition,
     emptyToFillingAction},
    // From FILLING to PARTIAL_FILLED when stopped before full
    {WL_STATE(FILLING), WL_STATE(PARTIAL_FILLED), stopCondition,
     stopActionClearFlag},
    // From PARTIAL_FILLED to FULL when the level reaches capacity
    {WL_STATE(PARTIAL_FILLED), WL_STATE(FULL), reachedFull, stopAction},
    // From FILLING to FULL when the level reaches capacity
    {WL_STATE(FILLING), WL_STATE(FULL), levelFullCondition, stopAction},

    // From FULL to DRAINING when start draining is commanded
    {WL_STATE(FULL), WL_STATE(DRAINING), startDrainingCondition,
     fullToDrainingAction},
    // From DRAINING to EMPTY when the level reaches 0
    {WL_STATE(DRAINING), WL_STATE(EMPTY), levelEmptyCondition, stopAction},
    // From DRAINING to PARTIAL_FILLED when stopped before empty
    {WL_STATE(DRAINING), WL_STATE(PARTIAL_FILLED), stopCondition,
     stopActionClearFlag},
    // From PARTIAL_FILLED to FILLING when start filling is commanded
    {WL_STATE(PARTIAL_FILLED), WL_STATE(FILLING), startFillingCondition,
     partialToFillingAction},
    // From PARTIAL_FILLED to DRAINING when start draining is commanded
    {WL_STATE(PARTIAL_FILLED), WL_STATE(DRAINING), startDrainingCondition,
     partialToDrainingAction},
};

} // namespace WL_SM

namespace SM {

class WaterLevelSM
    : public StateMachine<WaterLevelSM, StateWaterLevel, WaterLevelConfig,
                          WaterLevelData, WL_SM::WaterLevelTransitions,
                          sizeof(WL_SM::WaterLevelTransitions) /
                              sizeof(WL_SM::WaterLevelTransitions[0])> {
public:
  // Constructor with initial data
  WaterLevelSM(const WaterLevelData &initData)
      : StateMachine(StateWaterLevel::EMPTY, initData) {}

  // Default constructor
  constexpr WaterLevelSM()
      : StateMachine(StateWaterLevel::EMPTY, WaterLevelData{}) {}

  // Get the current state as a string for debugging or display
  String getStateString() const {
    switch (getState()) {
    case StateWaterLevel::EMPTY:
      return "EMPTY";
    case StateWaterLevel::FILLING:
      return "FILLING";
    case StateWaterLevel::DRAINING:
      return "DRAINING";
    case StateWaterLevel::PARTIAL_FILLED:
      return "PARTIAL_FILLED";
    case StateWaterLevel::FULL:
      return "FULL";
    case StateWaterLevel::OVERFLOW:
      return "OVERFLOW";
    default:
      return "UNKNOWN";
    }
  }
};

} // namespace SM

#endif // WATER_LEVEL_SM_HPP