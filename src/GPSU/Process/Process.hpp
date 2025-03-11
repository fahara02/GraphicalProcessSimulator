#ifndef PROCESS_HPP
#define PROCESS_HPP
#include "GUI/GUI.hpp"
#include "MCP23017.hpp"
#include "MenuSelector.hpp"
#include "PulseCounter.hpp"
#include "StateMachines/StepperMotorSM.hpp"
#include "StateMachines/TrafficLightSM.hpp"
#include "StateMachines/WaterLevelSM.hpp"
#include "driver/gpio.h"
#include <atomic>

namespace GPSU {

class Process {

public:
  Process();
  void init();
  void setupProcess(ProcessType type);
  void startProcess(ProcessType type);

protected:
  gpio_num_t sda_;
  gpio_num_t scl_;
  gpio_num_t reset_;
  gpio_num_t pinA_;
  gpio_num_t pinB_;
  gpio_num_t btn_;
  GUI::Display &display_;
  std::unique_ptr<COMPONENT::MCP23017> io_;
  std::unique_ptr<MenuSelector> menu_;
  std::unique_ptr<Pulse::Counter> counter_;
  std::unique_ptr<SM::TrafficLightSM> tlsm_;
  std::unique_ptr<SM::WaterLevelSM> wlsm_;
  std::unique_ptr<SM::StepperMotorSM> stsm_;
  ProcessType current_process_;
  MCP::Settings setting_;
  static StackType_t sharedStack[2048];
  static StaticTask_t sharedTCB;

private:
  static const MenuItem process_list[];
  static constexpr size_t process_count = 6;
  static void processTrafficLight(void *data);
  static void processWaterLevel(void *data);
  static void processStepperMotor(void *data);
  static void processStateMachine(void *data);
  static void processObjectCounter(void *data);
  static void processMotorControl(void *data);

  TaskHandle_t processTaskHandle = NULL;
  void create_task(ProcessType type);
  static void traffic_light_task(void *param);
  static void water_level_task(void *param);
  static void stepper_task(void *param);

  static void selectionChanged(size_t index, void *data);
  static void itemSelected(size_t index, void *data);
  void handleSelectionChanged(size_t index);
  void handleItemSelected(size_t index);
};

} // namespace GPSU
#endif