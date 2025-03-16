#include "Process.hpp"
// io_(std::make_unique<COMPONENT::MCP23017>(sda_, scl_, reset_)),
//  counter_(std::make_unique<Pulse::Counter>()),
// io_->configure(setting_);
// io_->pinMode(OUTPUT_OPEN_DRAIN, GPA1, GPA2, GPA3, GPA4);
// io_->pinMode(MCP::PORT::GPIOB, INPUT);
// io_->invertInput(true, GPB1, GPB2, GPB3, GPB4);
namespace GPSU {
StackType_t Process::sharedStack[2048]; // Define the stack array
StaticTask_t Process::sharedTCB;        // Define the TCB
} // namespace GPSU

namespace GPSU {

// Correct the process_list initialization
const MenuItem Process::process_list[] = {
    {"TRAFFIC_LIGHT", &Process::processTrafficLight},
    {"WATER_LEVEL", &Process::processWaterLevel},
    {"STEPPER_MOTOR", &Process::processStepperMotor},
    {"STATE_MACHINE", &Process::processStateMachine},
    {"OBJECT_COUNTER", &Process::processObjectCounter},
    {"MOTOR_CONTROL", &Process::processMotorControl}};
Process::Process()
    : sda_(GPIO_NUM_25), scl_(GPIO_NUM_33), reset_(GPIO_NUM_13),
      pinA_(GPIO_NUM_37), pinB_(GPIO_NUM_38), btn_(GPIO_NUM_32),
      display_(GUI::Display::getInstance(process_list, process_count)),
      // io_(std::make_unique<COMPONENT::MCP23017>(sda_, scl_, reset_)),
      menu_(std::make_unique<MenuSelector>(process_list, process_count, 4,
                                           pinA_, pinB_, btn_)),

      tlsm_(std::make_unique<SM::TrafficLightSM>()),
      wlsm_(std::make_unique<SM::WaterLevelSM>()),
      stsm_(std::make_unique<SM::StepperMotorSM>()),
      ocsm_(std::make_unique<SM::ObjectCounterSM>()),
      current_process_(ProcessType::ANY) {}

void Process::init() {

  display_.init();
  // io_->init();
  setting_.opMode = MCP::OperationMode::SequentialMode16;
  // io_->configure(setting_);
  // io_->pinMode(OUTPUT_OPEN_DRAIN, GPA1, GPA2, GPA3, GPA4);
  // io_->pinMode(MCP::PORT::GPIOB, INPUT);
  // io_->invertInput(true, GPB1, GPB2, GPB3, GPB4);
  menu_->set_selection_changed_cb(&Process::selectionChanged, this);
  menu_->set_item_selected_cb(&Process::itemSelected, this);
  menu_->init();
  GUI::Command cmd;
  cmd.type = GUI::CommandType::SHOW_MENU;
  display_.sendDisplayCommand(cmd);
}
void Process::selectionChanged(size_t index, void *data) {
  Process *proc = static_cast<Process *>(data);
  proc->handleSelectionChanged(index);
}

void Process::itemSelected(size_t index, void *data) {
  Process *proc = static_cast<Process *>(data);
  proc->handleItemSelected(index);
}
void Process::handleSelectionChanged(size_t index) {
  GUI::Command cmd;
  size_t selected_index = menu_->get_selected_index();
  display_.setCursorIndex(selected_index);
  cmd.cursor.index = selected_index;
  cmd.cursor.items = process_count;
  cmd.type = GUI::CommandType::SHOW_MENU;
  display_.sendDisplayCommand(cmd);
  if (current_process_ == ProcessType::ANY) {
    if (processTaskHandle != nullptr) {
      vTaskDelete(processTaskHandle);
      processTaskHandle = nullptr;
    }
  }
}
void Process::handleItemSelected(size_t index) {
  process_list[index].action(this);
  GUI::Command cmd;
  cmd.type = GUI::CommandType::SHOW_PROCESS_SCREEN;
  cmd.process_type = current_process_;
  display_.sendDisplayCommand(cmd);
}
void Process::initialiseProcess(ProcessType type) {
  switch (type) {
  case ProcessType::TRAFFIC_LIGHT:
    tlsm_->init();
    break;
  case ProcessType::WATER_LEVEL:
    wlsm_->init();
    break;
  case ProcessType::STEPPER_MOTOR:
    stsm_->init();
    break;
  case ProcessType::OBJECT_COUNTER:
    ocsm_->init();
    break;
  default:
    break;
  }
}

void Process::create_task(ProcessType type) {

  if (processTaskHandle != NULL) {
    vTaskDelete(processTaskHandle);
    processTaskHandle = NULL;
  }

  TaskFunction_t func = nullptr;
  const char *taskname = nullptr;

  switch (type) {
  case ProcessType::TRAFFIC_LIGHT:
    func = traffic_light_task;
    taskname = "traffic_task";
    break;
  case ProcessType::WATER_LEVEL:
    func = water_level_task;
    taskname = "water_level";
    break;
  case ProcessType::STEPPER_MOTOR:
    func = stepper_task;
    taskname = "stepper_motor";
    break;
  case ProcessType::OBJECT_COUNTER:
    func = object_counter_task;
    taskname = "object_counter";
    break;
  default:
    taskname = "Unknown";
  }
  if (func) {
    processTaskHandle = xTaskCreateStatic(func,        // Task function
                                          taskname,    // Task name
                                          2048,        // Stack depth
                                          this,        // Parameters
                                          1,           // Priority
                                          sharedStack, // Shared stack
                                          &sharedTCB); // Shared TCB
  }
}
void Process::startProcess(ProcessType type) {}

void Process::traffic_light_task(void *param) {
  auto instance = static_cast<GPSU::Process *>(param);
  TrafficLight::State state;
  TrafficLight::Context ctx;
  while (true) {

    ctx = instance->mapUserCommand<ProcessType::TRAFFIC_LIGHT,
                                   TrafficLight::Context>();

    // instance->tlsm_->updateData(ctx.inputs);
    instance->tlsm_->setAutoUpdate();
    instance->tlsm_->update();
    state = instance->tlsm_->current();
    const char *current =
        GPSU::Util::ToString::TLState(instance->tlsm_->current());
    const char *previous =
        GPSU::Util::ToString::TLState(instance->tlsm_->previous());

    Serial.printf("State changed to: %s from state %s  int time %lu\n", current,
                  previous, millis());
    GUI::Command cmd;
    cmd.type = GUI::CommandType::UPDATE_TRAFFIC_LIGHT;
    cmd.states.tl_state = state;
    cmd.inputs.tl_inputs = ctx.inputs;
    cmd.data.tl_data = ctx.data;
    instance->display_.sendDisplayCommand(cmd);

    vTaskDelay(pdMS_TO_TICKS(1000)); // Update every second
  }
}

void Process::water_level_task(void *param) {
  // auto instance = static_cast<GPSU::Process *>(param);
  // auto waterlevel = instance->wlsm_.get();
  // int level = 0;
  while (true) {
    // WaterLevel::State state;
    // state = waterlevel->current(); // Update state each iteration

    // if (state == WaterLevel::State::FILLING) { // Filling

    //   level = (level >= 100) ? 100 : level + 1;
    //   if (level == 100)
    //     state = WaterLevel::State::FULL;
    //   vTaskDelay(pdMS_TO_TICKS(100));
    // } else if (state == WaterLevel::State::FULL) { // Full

    //   vTaskDelay(pdMS_TO_TICKS(1000));
    //   state = WaterLevel::State::DRAINING;             // Transition to drain
    // } else if (state == WaterLevel::State::DRAINING) { // Draining

    //   level = (level <= 0) ? 0 : level - 1;
    //   if (level == 0) {
    //     state = 1;
    //   }

    //   vTaskDelay(pdMS_TO_TICKS(100));
    // }

    // if (state >= 0 || state <= 3) {

    //   tank_state.store(state);
    //   GUI::Command cmd;
    //   cmd.type = GUI::CommandType::UPDATE_WATER_LEVEL;
    //   cmd.water_level_state = state;
    //   cmd.analog_ch0 = level;
    //   display_.sendDisplayCommand(cmd);
    //   vTaskDelay(pdMS_TO_TICKS(100));
    // }
    vTaskDelay(1000);
  }
}
void Process::stepper_task(void *param) {
  while (true) {
    // int dir = 1;
    // int step = 0;
    // int angle = 45;
    // step = stepper_step;
    // step += 1;
    // if (angle > 360) {
    //   step = 1;
    //   angle = angle + step;
    //   dir = dir * (-1);
    // }

    // GUI::Command cmd;
    // cmd.type = GUI::CommandType::UPDATE_STEPPER;
    // stepper_step = step;
    // cmd.stteper_motor_state = 1; // running
    // cmd.analog_ch0 = dir;
    // cmd.analog_ch1 = step;
    // display_.sendDisplayCommand(cmd);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
void Process::object_counter_task(void *param) {
  auto instance = static_cast<GPSU::Process *>(param);
  ObjectCounter::State state;
  ObjectCounter::Context ctx;
  while (true) {

    // ctx = instance->mapUserCommand<ProcessType::TRAFFIC_LIGHT,
    //                                TrafficLight::Context>();

    // instance->tlsm_->updateData(ctx.inputs);
    instance->ocsm_->setAutoUpdate();
    instance->ocsm_->update();
    state = instance->ocsm_->current();
    const char *current =
        GPSU::Util::ToString::OCState(instance->ocsm_->current());
    const char *previous =
        GPSU::Util::ToString::OCState(instance->ocsm_->previous());

    Serial.printf("State changed to: %s from state %s  int time %lu\n", current,
                  previous, millis());
    GUI::Command cmd;
    cmd.type = GUI::CommandType::UPDATE_COUNTER;
    cmd.states.oc_state = state;
    cmd.inputs.oc_inputs = ctx.inputs;
    cmd.data.oc_data = ctx.data;
    instance->display_.sendDisplayCommand(cmd);

    vTaskDelay(pdMS_TO_TICKS(1000)); // Update every second
  }
}
//------------------------------------------------------------------------------
// Static callback functions for each menu item.
//------------------------------------------------------------------------------
void Process::processTrafficLight(void *data) {
  Process *proc = static_cast<Process *>(data);
  if (proc) {
    proc->current_process_ = ProcessType::TRAFFIC_LIGHT;
    proc->initialiseProcess(ProcessType::TRAFFIC_LIGHT);
    proc->display_.run_process(ProcessType::TRAFFIC_LIGHT);
    proc->create_task(ProcessType::TRAFFIC_LIGHT);
  }
}

void Process::processWaterLevel(void *data) {
  Process *proc = static_cast<Process *>(data);
  if (proc) {
    proc->current_process_ = ProcessType::WATER_LEVEL;
    proc->display_.run_process(ProcessType::WATER_LEVEL);
    proc->create_task(ProcessType::WATER_LEVEL);
  }
}

void Process::processStepperMotor(void *data) {
  Process *proc = static_cast<Process *>(data);
  if (proc) {
    proc->current_process_ = ProcessType::STEPPER_MOTOR;
    proc->display_.run_process(ProcessType::STEPPER_MOTOR);
    proc->create_task(ProcessType::STEPPER_MOTOR);
  }
}

void Process::processStateMachine(void *data) {
  Process *proc = static_cast<Process *>(data);
  if (proc) {
    proc->current_process_ = ProcessType::STATE_MACHINE;
    proc->display_.run_process(GPSU::ProcessType::STATE_MACHINE);
  }
}

void Process::processObjectCounter(void *data) {
  Process *proc = static_cast<Process *>(data);
  if (proc) {
    proc->current_process_ = ProcessType::OBJECT_COUNTER;
    proc->display_.run_process(GPSU::ProcessType::OBJECT_COUNTER);
    proc->create_task(ProcessType::OBJECT_COUNTER);
  }
}

void Process::processMotorControl(void *data) {
  Process *proc = static_cast<Process *>(data);
  if (proc) {
    proc->current_process_ = ProcessType::MOTOR_CONTROL;
    proc->display_.run_process(GPSU::ProcessType::MOTOR_CONTROL);
  }
}
} // namespace GPSU