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
struct WaterLevelData {
  int sensorADC;      // Raw sensor reading
  int pumpSpeed;      // Desired pump speed (positive for filling, negative for
                      // draining, 0 for off)
  float currentLevel; // Current water level in the tank in litre
  float fullLevel;    // Tank capacity, set from config.tankCapacity
  int pumpFlowRate;   // Pump flow rate, set from config.pumpFlowRate
  bool start_filling_flag;  // Flag to start filling
  bool start_draining_flag; // Flag to start draining
  bool stop_flag;           // Flag to stop the pump
  bool open_drain_valve;
  bool alarm;
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