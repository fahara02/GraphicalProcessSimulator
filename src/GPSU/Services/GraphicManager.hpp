#ifndef SERVICES_HPP
#define SERVICES_HPP

#include "MenuSelector.hpp"
#include <TFT_eSPI.h>
#include <memory>

// Define the GPIO pins for Channel A, Channel B, and the button.
gpio_num_t pinA = GPIO_NUM_37;
gpio_num_t pinB = GPIO_NUM_38;
gpio_num_t btn = GPIO_NUM_32;

namespace Service {

class Display {
public:
  // Singleton accessor.
  static Display &getInstance() {
    static Display instance;
    return instance;
  }

  // Delete copy constructor and assignment operator.
  Display(const Display &) = delete;
  Display &operator=(const Display &) = delete;

  // Accessor functions.
  TFT_eSPI &TFT() { return *tft_; }
  TFT_eSprite &Sprite() { return *sprite_; }

  // Function to show the menu.
  void show_menu() {
    // Implementation to display the menu (if needed).
  }

  // Static callback functions for each menu item.
  static void processTrafficLight() {
    getInstance().run_process(GPSU::ProcessType::TRAFFIC_LIGHT);
  }
  static void processWaterLevel() {
    getInstance().run_process(GPSU::ProcessType::WATER_LEVEL);
  }
  static void processStepperMotorControl() {
    getInstance().run_process(GPSU::ProcessType::STEPPER_MOTOR_CONTROL);
  }
  static void processStateMachine() {
    getInstance().run_process(GPSU::ProcessType::STATE_MACHINE);
  }
  static void processObjectCounter() {
    getInstance().run_process(GPSU::ProcessType::OBJECT_COUNTER);
  }
  static void processMotorControl() {
    getInstance().run_process(GPSU::ProcessType::MOTOR_CONTROl);
  }
  void init() {
    if (!initialised) {
      menu_->set_selection_changed_Cb(onSelectionChanged);
      menu_->set_item_selected_cb(onItemSelected);
      menu_->init();
      tft_->init();
      initialised = true;
    }
  }

private:
  explicit Display()
      : tft_(std::make_unique<TFT_eSPI>()),
        sprite_(std::make_unique<TFT_eSprite>(tft_.get())),
        menu_(std::make_unique<MenuSelector>(menuItems, MENU_ITEM_COUNT, 4,
                                             pinA, pinB, btn)) {}

  // Runs a process based on the ProcessType.
  void run_process(GPSU::ProcessType type) {
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

  // Static callback for when the menu selection changes.
  static void onSelectionChanged(size_t index) {
    Serial.print("Selected: ");
    Serial.println(menuItems[index].label);
  }

  // Static callback for when a menu item is confirmed.
  static void onItemSelected(size_t index) { Serial.println("Item confirmed"); }

  // Private member variables.
  bool initialised = false;
  std::unique_ptr<TFT_eSPI> tft_;
  std::unique_ptr<TFT_eSprite> sprite_;
  std::unique_ptr<MenuSelector> menu_;

  // Static menu items array.
  inline static const MenuItem menuItems[] = {
      {"TRAFFIC_LIGHT", processTrafficLight},
      {"WATER_LEVEL", processWaterLevel},
      {"STEPPER_MOTOR_CONTROL", processStepperMotorControl},
      {"STATE_MACHINE", processStateMachine},
      {"OBJECT_COUNTER", processObjectCounter},
      {"MOTOR_CONTROl", processMotorControl}};
  inline static constexpr size_t MENU_ITEM_COUNT =
      sizeof(menuItems) / sizeof(menuItems[0]);
};

} // namespace Service

#endif // SERVICES_HPP
