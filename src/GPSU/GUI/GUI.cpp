#include "GUI/GUI.hpp"
namespace GUI {
gpio_num_t pinA = GPIO_NUM_37;
gpio_num_t pinB = GPIO_NUM_38;
gpio_num_t btn = GPIO_NUM_32;
const MenuItem Display::menuItems[] = {
    {"TRAFFIC_LIGHT", Display::processTrafficLight},
    {"WATER_LEVEL", Display::processWaterLevel},
    {"STEPPER_MOTOR_CONTROL", Display::processStepperMotorControl},
    {"STATE_MACHINE", Display::processStateMachine},
    {"OBJECT_COUNTER", Display::processObjectCounter},
    {"MOTOR_CONTROl", Display::processMotorControl}};
Display &Display::getInstance() {
  static Display instance;
  return instance;
}
Display::Display()
    : initialised(false), tft_(std::make_unique<TFT_eSPI>()),
      sprite_(std::make_unique<TFT_eSprite>(tft_.get())),
      menu_(std::make_unique<MenuSelector>(menuItems, MENU_ITEM_COUNT, 4, pinA,
                                           pinB, btn)),
      current_process_(GPSU::ProcessType::ANY) {}
//------------------------------------------------------------------------------
// Accessor functions
//------------------------------------------------------------------------------
TFT_eSPI &Display::TFT() { return *tft_; }

TFT_eSprite &Display::Sprite() { return *sprite_; }
//------------------------------------------------------------------------------
// show_menu (currently empty)
//------------------------------------------------------------------------------
void Display::show_menu() {
  // Implementation of showing menu goes here.
}

//------------------------------------------------------------------------------
// Static callback functions for each menu item.
//------------------------------------------------------------------------------
void Display::processTrafficLight() {
  getInstance().run_process(GPSU::ProcessType::TRAFFIC_LIGHT);
}

void Display::processWaterLevel() {
  getInstance().run_process(GPSU::ProcessType::WATER_LEVEL);
}

void Display::processStepperMotorControl() {
  getInstance().run_process(GPSU::ProcessType::STEPPER_MOTOR_CONTROL);
}

void Display::processStateMachine() {
  getInstance().run_process(GPSU::ProcessType::STATE_MACHINE);
}

void Display::processObjectCounter() {
  getInstance().run_process(GPSU::ProcessType::OBJECT_COUNTER);
}

void Display::processMotorControl() {
  getInstance().run_process(GPSU::ProcessType::MOTOR_CONTROl);
}

//------------------------------------------------------------------------------
// Initialisation function.
//------------------------------------------------------------------------------
void Display::init() {
  if (!initialised) {
    menu_->set_selection_changed_Cb(onSelectionChanged);
    menu_->set_item_selected_cb(onItemSelected);
    menu_->init();
    tft_->init();
    initialised = true;
  }
}

//------------------------------------------------------------------------------
// Runs a process based on the ProcessType.
//------------------------------------------------------------------------------
void Display::run_process(GPSU::ProcessType type) {
  current_process_ = type;
  switch (type) {
  case GPSU::ProcessType::TRAFFIC_LIGHT:
    Serial.println("Running traffic light process");
    break;
  case GPSU::ProcessType::WATER_LEVEL:
    Serial.println("Running water level process");
    break;
  case GPSU::ProcessType::STEPPER_MOTOR_CONTROL:
    Serial.println("Running stepper motor process");
    break;
  case GPSU::ProcessType::STATE_MACHINE:
    Serial.println("Running state machine process");
    break;
  case GPSU::ProcessType::OBJECT_COUNTER:
    Serial.println("Running object counter process");
    break;
  case GPSU::ProcessType::MOTOR_CONTROl:
    Serial.println("Running motor control process");
    break;
  default:
    Serial.println("Unknown process");
    break;
  }
}

//------------------------------------------------------------------------------
// Static callback for when the menu selection changes.
//------------------------------------------------------------------------------
void Display::onSelectionChanged(size_t index) {
  Serial.print("Selected: ");
  Serial.println(menuItems[index].label);
}

//------------------------------------------------------------------------------
// Static callback for when a menu item is confirmed.
//------------------------------------------------------------------------------
void Display::onItemSelected(size_t index) { Serial.println("Item confirmed"); }
} // namespace GUI