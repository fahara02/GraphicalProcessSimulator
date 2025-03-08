#ifndef STATE_DEFINES_HPP
#define STATE_DEFINES_HPP
#include "stdint.h"
namespace SM {

enum class StateTrafficLight { INIT, RED_STATE, GREEN_STATE, YELLOW_STATE };
struct TrafficLightConfig {
  int defaultRedTimeout_ms = 5000;
  bool allowImmediateTransition = false;
  uint16_t error_blink_interval = 500;
  uint8_t max_errors = 3;
};
struct TrafficLightData {
  int redTimeout_ms;
  int greenTimeout_ms;
  int yellowTimeout_ms;
  int timer_ms;
  bool button_pressed;
};
enum class StateWaterLevel : uint8_t {
  EMPTY = 0,
  FILLING,
  DRAINING,
  PARTIAL_FILLED,
  FULL,
  OVERFLOW,

};
struct WaterLevelConfig {
  const uint16_t tankCapacity = 2000;
  const uint16_t pumpFlowRate = 100;
  const float sensor_volt_per_litre_mv = 2.3;
};
struct WaterLevelData {
  int sensorADC;
  int pumpSpeed;
  float currentLevel;
};
struct StapperConfig {
  const float degreePerStep = 1.9;
  const uint16_t step_per_mm = 120;
};
struct StepperData {
  uint16_t currentDirection = 1; //+1 or -1
  uint16_t currentStep;
};

enum class StateStepperMotor : uint8_t {
  IDLE = 0,
  RUN,
  ERROR

};

} // namespace SM

#endif