#ifndef TRAFFIC_LIGHT_SM_HPP
#define TRAFFIC_LIGHT_SM_HPP
#include "Arduino.h"

#include "StateMachines/StateMachine.hpp"
#define TL_STATE(state) static_cast<uint8_t>(SM::StateTrafficLight::state)

namespace SM {
class TrafficLightSM; // Forward declaration
} // namespace SM

namespace TL_SM {
static void emitEvent(SM::StateTrafficLight state) {}

bool alwaysTrue(const SM::TrafficLightData &data,
                const SM::TrafficLightConfig &config) {
  return true;
}
bool timeoutRed(const SM::TrafficLightData &data,
                const SM::TrafficLightConfig &config) {
  return data.timer_ms >= data.redTimeout_ms;
}
bool timeoutGreen(const SM::TrafficLightData &data,
                  const SM::TrafficLightConfig &config) {
  return data.timer_ms >= data.greenTimeout_ms;
}
bool timeoutYellow(const SM::TrafficLightData &data,
                   const SM::TrafficLightConfig &config) {
  return data.timer_ms >= data.yellowTimeout_ms;
}
bool buttonPress(const SM::TrafficLightData &data,
                 const SM::TrafficLightConfig &config) {
  return data.button_pressed;
}

void initToRed(SM::TrafficLightData &data,
               const SM::TrafficLightConfig &config) {
  emitEvent(SM::StateTrafficLight::RED_STATE);
  data.timer_ms = 0;
}
void redToGreen(SM::TrafficLightData &data,
                const SM::TrafficLightConfig &config) {
  emitEvent(SM::StateTrafficLight::GREEN_STATE);
  data.timer_ms = 0;
}
void greenToYellow(SM::TrafficLightData &data,
                   const SM::TrafficLightConfig &config) {
  emitEvent(SM::StateTrafficLight::YELLOW_STATE);
  data.timer_ms = 0;
}
void yellowToRed(SM::TrafficLightData &data,
                 const SM::TrafficLightConfig &config) {
  emitEvent(SM::StateTrafficLight::RED_STATE);
  data.timer_ms = 0;
}

constexpr SM::Transition<SM::TrafficLightData, SM::TrafficLightConfig>
    TrafficTransitions[] = {
        {TL_STATE(INIT), TL_STATE(RED_STATE), alwaysTrue, initToRed},
        {TL_STATE(RED_STATE), TL_STATE(GREEN_STATE), timeoutRed, redToGreen},
        {TL_STATE(GREEN_STATE), TL_STATE(YELLOW_STATE), timeoutGreen,
         greenToYellow},
        {TL_STATE(YELLOW_STATE), TL_STATE(RED_STATE), timeoutYellow,
         yellowToRed},
        {TL_STATE(RED_STATE), TL_STATE(GREEN_STATE), buttonPress, redToGreen}};

} // namespace TL_SM

namespace SM {

class TrafficLightSM
    : public StateMachine<TrafficLightSM, StateTrafficLight, TrafficLightConfig,
                          TrafficLightData, TL_SM::TrafficTransitions,
                          sizeof(TL_SM::TrafficTransitions) /
                              sizeof(TL_SM::TrafficTransitions[0])> {
public:
  TrafficLightSM(const TrafficLightData &initData)
      : StateMachine(StateTrafficLight::INIT, initData) {}

  // Default constructor remains if needed
  constexpr TrafficLightSM()
      : StateMachine(StateTrafficLight::INIT, TrafficLightData{}) {}

  void incrementTimer(int delta_ms) { data_.timer_ms += delta_ms; }

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