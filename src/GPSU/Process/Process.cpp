// #include "Process.hpp"

// namespace Process {
// StackType_t Display::sharedStack[2048]; // Define the stack array
// StaticTask_t Display::sharedTCB;        // Define the TCB
// } // namespace Process

// namespace Process {
// const MenuItem Process::all_process[] = {
//     {"TRAFFIC_LIGHT", Process::processTrafficLight},
//     {"WATER_LEVEL", Process::processWaterLevel},
//     {"STEPPER_MOTOR", Process::processStepperMotor},
//     {"STATE_MACHINE", Process::processStateMachine},
//     {"OBJECT_COUNTER", Process::processObjectCounter},
//     {"MOTOR_CONTROl", Process::processMotorControl}};
// Process::Process()
//     : sda_(GPIO_NUM_25), scl_(GPIO_NUM_33), reset_(GPIO_NUM_13),
//       pinA_(GPIO_NUM_37), pinB_(GPIO_NUM_38), btn_(GPIO_NUM_32),
//       display_(GUI::Display::getInstance()),
//       io_expander(std::make_unique<COMPONENET::MCP23017>(sda_, scl_,
//       reset_)), menu_selector_(std::make_unique<MenuSelector>(
//           all_process, MENU_PRCOESS_COUNT, 4, pinA_, pinB_, btn_)),
// {}

// void Process::init() {
//   display_.init();
//   setting_.opMode = MCP::OperationMode::SequentialMode16;
//   io_expander->configure(setting_);
//   io_expander->pinMode(OUTPUT_OPEN_DRAIN, GPA1, GPA2, GPA3, GPA4);
//   io_expander->pinMode(MCP::PORT::GPIOB, INPUT);
//   io_expander->invertInput(true, GPB1, GPB2, GPB3, GPB4);

//   menu_selector_->set_selection_changed_Cb([](size_t index) {
//     GUI::DisplayCommand cmd;
//     cmd.type = GUI::DisplayCommandType::SHOW_MENU;
//     getInstance().sendDisplayCommand(cmd);
//     if (getInstance().current_process_ == ProcessType::ANY) {

//       if (display_.processTaskHandle != NULL) {
//         vTaskDelete(display.processTaskHandle);
//         display_.processTaskHandle = NULL;
//       }
//     }
//   });
//   menu_->set_item_selected_cb([](size_t index) {
//     menuItems[index].action(); // Sets current_process_
//     GUI::DisplayCommand cmd;
//     cmd.type = DisplayCommandType::SHOW_PROCESS_SCREEN;
//     cmd.process_type = getInstance().current_process_;
//     getInstance().sendDisplayCommand(cmd);
//   });
// }
// void Process::startProcess(ProcessType type) {
//   // Delete the existing task, if any
//   if (processTaskHandle != NULL) {
//     vTaskDelete(processTaskHandle);
//     processTaskHandle = NULL; // Clear the handle
//   }

//   // Create the new task using the shared static memory
//   if (type == ProcessType::TRAFFIC_LIGHT) {
//     processTaskHandle = xTaskCreateStatic(traffic_light_task, // Task
//     function
//                                           "traffic_task",     // Task name
//                                           2048,               // Stack depth
//                                           NULL,               // Parameters
//                                           1,                  // Priority
//                                           sharedStack,        // Shared stack
//                                           &sharedTCB          // Shared TCB
//     );
//   } else if (type == ProcessType::WATER_LEVEL) {
//     processTaskHandle = xTaskCreateStatic(water_level_task, // Task function
//                                           "water_level",    // Task name
//                                           2048,             // Stack depth
//                                           NULL,             // Parameters
//                                           1,                // Priority
//                                           sharedStack,      // Shared stack
//                                           &sharedTCB        // Shared TCB
//     );
//   } else if (type == ProcessType::STEPPER_MOTOR_CONTROL) {
//     processTaskHandle = xTaskCreateStatic(stepper_task,    // Task function
//                                           "stepper_motor", // Task name
//                                           2048,            // Stack depth
//                                           NULL,            // Parameters
//                                           1,               // Priority
//                                           sharedStack,     // Shared stack
//                                           &sharedTCB       // Shared TCB
//     );
//   }
// }

// void Process::traffic_light_task(void *param) {
//   int state = 0;
//   while (true) {
//     GUI::DisplayCommand cmd;
//     cmd.type = GUI::DisplayCommandType::UPDATE_TRAFFIC_LIGHT;
//     cmd.traffic_light_state = state;
//     display_.sendDisplayCommand(cmd);
//     state = (state + 1) % 3;         // Cycle through 0, 1, 2
//     vTaskDelay(pdMS_TO_TICKS(1000)); // Update every second
//   }
// }

// void Process::water_level_task(void *param) {
//   int level = 0;
//   while (true) {
//     int state = 0;
//     state = tank_state.load(); // Update state each iteration

//     if (state == 1) { // Filling

//       level = (level >= 100) ? 100 : level + 1;
//       if (level == 100)
//         state = 2;
//       vTaskDelay(pdMS_TO_TICKS(100));
//     } else if (state == 2) { // Full

//       vTaskDelay(pdMS_TO_TICKS(1000));
//       state = 0;             // Transition to drain
//     } else if (state == 0) { // Draining

//       level = (level <= 0) ? 0 : level - 1;
//       if (level == 0) {
//         state = 1;
//       }

//       vTaskDelay(pdMS_TO_TICKS(100));
//     }

//     if (state >= 0 || state <= 3) {

//       tank_state.store(state);
//       DisplayCommand cmd;
//       cmd.type = DisplayCommandType::UPDATE_WATER_LEVEL;
//       cmd.water_level_state = state;
//       cmd.analog_ch0 = level;
//       Display::getInstance().sendDisplayCommand(cmd);
//       vTaskDelay(pdMS_TO_TICKS(100));
//     }
//   }
// }
// void Process::stepper_task(void *param) {
//   while (true) {
//     int dir = 1;
//     int step = 0;
//     int angle = 45;
//     step = stepper_step;
//     step += 1;
//     if (angle > 360) {
//       step = 1;
//       angle = angle + step;
//       dir = dir * (-1);
//     }

//     GUI::DisplayCommand cmd;
//     cmd.type = GUI::DisplayCommandType::UPDATE_STEPPER;
//     stepper_step = step;
//     cmd.stteper_motor_state = 1; // running
//     cmd.analog_ch0 = dir;
//     cmd.analog_ch1 = step;
//     display_.sendDisplayCommand(cmd);
//     vTaskDelay(pdMS_TO_TICKS(100));
//   }
// }

// } // namespace Process