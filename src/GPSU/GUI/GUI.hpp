#ifndef GUI_HPP
#define GUI_HPP

#include "Assets/AssetTrafficLight.hpp"
#include "GUIConstants.hpp"
#include "MenuSelector.hpp"
#include "Process/ProcessDefines.hpp"
#include <TFT_eSPI.h>
#include <memory>
#include <tuple>

namespace GUI {
enum class DisplayCommandType {
  SHOW_MENU,            // Display the main menu
  SHOW_PROCESS_SCREEN,  // Show the initial screen for a selected process
  UPDATE_TRAFFIC_LIGHT, // Update traffic light display with a state

};

struct DisplayCommand {
  DisplayCommandType type;
  union {
    GPSU::ProcessType process_type; // For SHOW_PROCESS_SCREEN
    int traffic_light_state; // For UPDATE_TRAFFIC_LIGHT (e.g., 0=red, 1=yellow,
                             // 2=green)
  };
};
class Display {
public:
  static Display &getInstance();

  Display(const Display &) = delete;
  Display &operator=(const Display &) = delete;

  TFT_eSPI &Canvas();
  TFT_eSprite &Sprite();

  void init();

private:
  // Private constructor.
  explicit Display();
  bool initialised;
  std::unique_ptr<TFT_eSPI> canvas_;
  std::unique_ptr<TFT_eSprite> bg_;
  std::unique_ptr<TFT_eSprite> img_;
  std::unique_ptr<TFT_eSprite> frame_;
  std::unique_ptr<MenuSelector> menu_;
  GPSU::ProcessType current_process_;
  QueueHandle_t displayQueue;

  void showMenu();
  void showProcessScreen(GPSU::ProcessType type);
  void updateTrafficLightDisplay(int state);
  void sendDisplayCommand(const DisplayCommand &cmd);
  void run_process(GPSU::ProcessType type);
  void showSubMenu();
  static void processTrafficLight();
  static void processWaterLevel();
  static void processStepperMotorControl();
  static void processStateMachine();
  static void processObjectCounter();
  static void processMotorControl();

  static void onSelectionChanged(size_t index);
  static void onItemSelected(size_t index);
  static void display_loop(void *param);
  static void traffic_light_task(void *param);

  static const MenuItem menuItems[];
  static constexpr size_t MENU_ITEM_COUNT = 6;

  // helpers
  void drawAlignText(AlignMent align, const char *text, uint8_t font,
                     Colors txt = Colors::main, Colors bg = Colors::black);
  std::tuple<int16_t, int16_t>
  calculateAlignment(AlignMent align, int16_t Width, int16_t Height);
};

} // namespace GUI

#endif // GUI_HPP
