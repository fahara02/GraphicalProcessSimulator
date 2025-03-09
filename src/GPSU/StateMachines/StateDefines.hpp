#ifndef STATE_DEFINES_HPP
#define STATE_DEFINES_HPP
#include "stdint.h"
namespace SM {

enum class StateTrafficLight { INIT, RED_STATE, GREEN_STATE, YELLOW_STATE };
struct TrafficLightConfig {
  int redTimeout_ms = 5000;
  int greenTimeout_ms = 3000;
  int yellowTimeout_ms = 2000;
  bool allowImmediateTransition = false;
  uint16_t error_blink_interval = 500;
  uint8_t max_errors = 3;
};
enum class CommandsTL {
  NONE,
  RESET,
  TURN_ON_RED,
  TURN_ON_YELLOW,
  TURN_ON_GREEN,
};
struct TrafficLightInput {
  int delta_time_ms;
  bool button_pressed = false;
};
struct TrafficLightOutPut {
  int timeout_ms;
  bool immediate_transition = false;
};
struct TLCommand {
  CommandsTL command;
  TrafficLightOutPut data;
};
struct TrafficLightData {
  uint32_t current_time_ms;
};

enum class StateWaterLevel : uint8_t {
  EMPTY = 0,
  START_FILLING,
  FILLING,
  DRAINING,
  PARTIAL_FILLED,
  FULL,
  OVERFLOW,

};
struct WaterLevelConfig {
  uint16_t tankMaxCapacityLitre = 2100;
  uint16_t tankCapacityLitre = 2000;
  uint16_t partialLevelLow = 500;
  uint16_t partialLevelHigh = 1900;
  uint16_t drainingMark = 1800;
  int pumpFillFlowRate = 100;
  int pumpDrainFlowRate = -50;
  float sensorVoltperLitre = 2.3;
  float sensorMinSensivityLitre = 10;
};
enum class CommandsWL {
  NONE,
  LEVEL_UPDATE,
  START_FILL,
  FILL_TO_LEVEL,
  STOP,
  START_DRAIN,
  TRIGGER_ALARM,

};
struct WaterLevelOutputs {
  int pumpSpeed;
  float targetLevel;
  bool open_drain_valve;
  bool alarm;
};
struct WaterLevelInputs {
  int sensorADC;
  float currentLevel;
  float targetLevel;
  bool start_filling_flag;
  bool start_draining_flag;
  bool stop_flag;
};

struct WLCommand {
  CommandsWL command;
  WaterLevelOutputs data;
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