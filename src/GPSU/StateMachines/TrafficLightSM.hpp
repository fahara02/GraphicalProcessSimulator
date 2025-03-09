#ifndef TRAFFIC_LIGHT_SM_HPP
#define TRAFFIC_LIGHT_SM_HPP
#include "Arduino.h"

#include "StateMachines/StateMachine.hpp"
#define TL_STATE(state) static_cast<uint8_t>(SM::StateTrafficLight::state)

namespace SM {
class TrafficLightSM; // Forward declaration
} // namespace SM

namespace TL_SM {

inline bool alwaysTrue(const SM::StateTrafficLight oldState,
                       const SM::TrafficLightInput &data,
                       const SM::TrafficLightConfig &config) {
  return true;
}
inline bool timeoutRed(const SM::StateTrafficLight oldState,
                       const SM::TrafficLightInput &data,
                       const SM::TrafficLightConfig &config) {
  return data.current_time_ms >= config.redTimeout_ms;
}
inline bool timeoutGreen(const SM::StateTrafficLight oldState,
                         const SM::TrafficLightInput &data,
                         const SM::TrafficLightConfig &config) {
  return data.current_time_ms >= config.greenTimeout_ms;
}
inline bool timeoutYellowToRed(const SM::StateTrafficLight oldState,
                               const SM::TrafficLightInput &data,
                               const SM::TrafficLightConfig &config) {
  return (data.current_time_ms >= config.yellowTimeout_ms) &&
         (oldState == SM::StateTrafficLight::GREEN_STATE);
}
inline bool timeoutYellowToGreen(const SM::StateTrafficLight oldState,
                                 const SM::TrafficLightInput &data,
                                 const SM::TrafficLightConfig &config) {
  return (data.current_time_ms >= config.yellowTimeout_ms) &&
         (oldState == SM::StateTrafficLight::RED_STATE);
}
inline bool buttonPress(const SM::StateTrafficLight oldState,
                        const SM::TrafficLightInput &data,
                        const SM::TrafficLightConfig &config) {
  return data.button_pressed;
}

inline SM::TLCommand initToRed(const SM::TrafficLightInput &data,
                               const SM::TrafficLightConfig &config) {
  SM::TLCommand cmd;
  cmd.command = SM::CommandsTL::TURN_ON_RED;

  cmd.data.timer_ms = 0;
  cmd.data.immediate_transition = false;
  return cmd;
}
inline SM::TLCommand redToYellow(const SM::TrafficLightInput &data,
                                 const SM::TrafficLightConfig &config) {

  SM::TLCommand cmd;
  cmd.command = SM::CommandsTL::TURN_ON_YELLOW;

  cmd.data.timer_ms = config.redTimeout_ms;
  return cmd;
}
inline SM::TLCommand redToGreen(const SM::TrafficLightInput &data,
                                const SM::TrafficLightConfig &config) {

  SM::TLCommand cmd;
  cmd.command = SM::CommandsTL::TURN_ON_GREEN;
  cmd.data.immediate_transition = true;
  return cmd;
}
inline SM::TLCommand yellowToGreen(const SM::TrafficLightInput &data,
                                   const SM::TrafficLightConfig &config) {
  SM::TLCommand cmd;
  cmd.command = SM::CommandsTL::TURN_ON_GREEN;

  cmd.data.timer_ms = config.yellowTimeout_ms;
  return cmd;
}
inline SM::TLCommand greenToYellow(const SM::TrafficLightInput &data,
                                   const SM::TrafficLightConfig &config) {
  SM::TLCommand cmd;
  cmd.command = SM::CommandsTL::TURN_ON_YELLOW;

  cmd.data.timer_ms = config.greenTimeout_ms;
  return cmd;
}
inline SM::TLCommand yellowToRed(const SM::TrafficLightInput &data,
                                 const SM::TrafficLightConfig &config) {
  SM::TLCommand cmd;
  cmd.command = SM::CommandsTL::TURN_ON_RED;

  cmd.data.timer_ms = config.yellowTimeout_ms;
  return cmd;
}

constexpr SM::Transition<SM::StateTrafficLight, SM::TrafficLightConfig,
                         SM::TrafficLightInput, SM::TLCommand>
    TrafficTransitions[] = {
        {TL_STATE(INIT), TL_STATE(RED_STATE), alwaysTrue, initToRed},
        {TL_STATE(RED_STATE), TL_STATE(YELLOW_STATE), timeoutRed, redToYellow},
        {TL_STATE(YELLOW_STATE), TL_STATE(GREEN_STATE), timeoutYellowToGreen,
         yellowToGreen},
        {TL_STATE(GREEN_STATE), TL_STATE(YELLOW_STATE), timeoutGreen,
         greenToYellow},
        {TL_STATE(YELLOW_STATE), TL_STATE(RED_STATE), timeoutYellowToRed,
         yellowToRed},
        {TL_STATE(RED_STATE), TL_STATE(GREEN_STATE), buttonPress, redToGreen}};

} // namespace TL_SM

namespace SM {

class TrafficLightSM
    : public StateMachine<TrafficLightSM, StateTrafficLight, TrafficLightConfig,
                          TrafficLightInput, TLCommand, CommandsTL,
                          TL_SM::TrafficTransitions,
                          sizeof(TL_SM::TrafficTransitions) /
                              sizeof(TL_SM::TrafficTransitions[0])> {
public:
  TrafficLightSM(const TrafficLightInput &initData)
      : StateMachine(StateTrafficLight::INIT, TrafficLightConfig{}, initData) {}
  TrafficLightSM(const TrafficLightConfig config)
      : StateMachine(StateTrafficLight::INIT, config, TrafficLightInput{}) {}
  TrafficLightSM(const TrafficLightConfig config,
                 const TrafficLightInput &initData)
      : StateMachine(StateTrafficLight::INIT, config, initData) {}

  // Default constructor remains if needed
  constexpr TrafficLightSM()
      : StateMachine(StateTrafficLight::INIT, TrafficLightConfig{},
                     TrafficLightInput{}) {}

  void incrementTimer(int delta_ms) { inputData_.current_time_ms += delta_ms; }

  // Add method to get state as a string
  String getStateString() const {
    switch (getState()) {
    case StateTrafficLight::INIT:
      return "INIT";
    case StateTrafficLight::RED_STATE:
      return "RED";
    case StateTrafficLight::GREEN_STATE:
      return "GREEN";
    case StateTrafficLight::YELLOW_STATE:
      return "YELLOW";
    default:
      return "UNKNOWN";
    }
  }
};

} // namespace SM

#endif