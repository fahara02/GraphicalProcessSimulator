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
  using tlContext = TrafficLight::Context;
  using tlMode = TrafficLight::Mode;
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
  // std::unique_ptr<Pulse::Counter> counter_;
  std::unique_ptr<SM::TrafficLightSM> tlsm_;
  std::unique_ptr<SM::WaterLevelSM> wlsm_;
  std::unique_ptr<SM::StepperMotorSM> stsm_;
  ProcessType current_process_;
  MCP::Settings setting_;
  static StackType_t sharedStack[2048];
  static StaticTask_t sharedTCB;

private:
  TrafficLight ::Context tl_ctx = TrafficLight ::Context{
      TrafficLight::State::INIT, // previous_state
      {5000, 3000, 2000, false}, // config
      {0},                       // data
      {{0}, {false}},            // inputs
      TrafficLight::Event::OK,   // Event
      TrafficLight::Mode::MANUAL // Mode
  };
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
  void initialiseProcess(ProcessType type);
  template <ProcessType type, typename Context> Context mapUserCommand() {

    if (type == ProcessType::TRAFFIC_LIGHT) {
      Context ctx;
      uint8_t pinStatus = io_->digitalRead(MCP::PORT::GPIOB);

      ctx.inputs.user_command.turn_on_red = (pinStatus >> 1) & 0x01;    // GPB1
      ctx.inputs.user_command.turn_on_green = (pinStatus >> 2) & 0x01;  // GPB2
      ctx.inputs.user_command.turn_on_yellow = (pinStatus >> 3) & 0x01; // GPB3
      ctx.inputs.user_command.button_pressed = (pinStatus >> 4) & 0x01; // GPB4
      ctx.inputs.new_input = true;
      return ctx;
    }
    return Context{};
  }
};

} // namespace GPSU
#endif