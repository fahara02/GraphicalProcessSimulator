// #ifndef PROCESS_HPP
// #define PROCESS_HPP
// #include "GUI/GUI.hpp"
// #include "MCP23017.hpp"
// #include "MenuSelector.hpp"
// #include "PulseCounter.hpp"
// #include "StateMachines/StateMachines.hpp"
// #include "driver/gpio.h"
// #include <atomic>

// namespace GPSU {

// class Process {

// public:
//   Process();
//   void init();
//   void setupProcess(ProcessType type);
//   void startProcess(ProcessType type);

// protected:
//   gpio_num_t sda_;
//   gpio_num_t scl_;
//   gpio_num_t reset_;
//   gpio_num_t pinA_;
//   gpio_num_t pinB_;
//   gpio_num_t btn_;
//   GUI::Display &display_;
//   std::unique_ptr<COMPONENET::MCP23017> io_expander_;
//   std::unique_ptr<MenuSelector> menu_selector_;
//   std::unique_ptr<Pulse::Counter> counter_;
//   ProcessType current_process_;
//   MCP::Settings setting_;

//   int stepper_step = 1;
//   std::atomic<int> tank_state{1};
//   size_t tank_capacity_litre = 2000;

//   static StackType_t sharedStack[2048];
//   static StaticTask_t sharedTCB;

// private:
//   static const MenuItem all_process[];
//   static constexpr size_t MENU_PRCOESS_COUNT = 6;

//   static void processTrafficLight();
//   static void processWaterLevel();
//   static void processStepperMotor();
//   static void processStateMachine();
//   static void processObjectCounter();
//   static void processMotorControl();

//   TaskHandle_t processTaskHandle = NULL;

//   static void traffic_light_task(void *param);
//   static void water_level_task(void *param);
//   static void stepper_task(void *param);
// };

// } // namespace GPSU
// #endif