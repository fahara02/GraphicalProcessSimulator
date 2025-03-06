#ifndef GUI_HPP
#define GUI_HPP

#include "Assets/AssetImages.hpp"
#include "GUIConstants.hpp"
#include "MenuSelector.hpp"
#include "Process/ProcessDefines.hpp"
#include "atomic"
#include <TFT_eSPI.h>
#include <memory>
#include <tuple>

namespace GUI {
enum class DisplayCommandType {
  SHOW_STARTUP,
  SHOW_MENU,            // Display the main menu
  SHOW_PROCESS_SCREEN,  // Show the initial screen for a selected process
  UPDATE_TRAFFIC_LIGHT, // Update traffic light display with a state
  UPDATE_WATER_LEVEL,
  UPDATE_STEPPER,
  UPDATE_COUNTER,
  UPDATE_STATE_MACHINE,
  UPDATE_MOTOR
};

struct DisplayCommand {
  DisplayCommandType type;
  union {
    GPSU::ProcessType process_type; // For SHOW_PROCESS_SCREEN
    int traffic_light_state;        //  (e.g., 0=red, 1=yellow,  2=green)
    int water_level_state; //  (e.g., 0=drained, 1=filling,  2=full,3=overflow)
    int stteper_motor_state;  //  (e.g., 0=cw, 1=ccw, 2=limit ,3=jitter)
    int object_counter_state; //  (e.g., 0=empty, 1=incr, 2=decr ,3=error)
    int state_machine_state;  //  (e.g., 0=init, 1=first, 2=second ,3=end)
    int motor_control_state;  //  (e.g., 0=run, 1=stop, 2=fwd ,3=rev)
  };
  int water_level;
  uint16_t analog_ch0;
  uint16_t analog_ch1;
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
  bool setup_traffic = false;
  bool setup_waterlevel = false;
  std::atomic<int> tank_state{1};
  int tank_capacity_litre = 2000;
  std::unique_ptr<TFT_eSPI> canvas_;
  std::unique_ptr<TFT_eSprite> bg_;
  std::unique_ptr<TFT_eSprite> label_;
  std::unique_ptr<TFT_eSprite> analog_;
  std::unique_ptr<TFT_eSprite> img_;
  std::unique_ptr<TFT_eSprite> frame_;
  std::unique_ptr<MenuSelector> menu_;
  GPSU::ProcessType current_process_;
  static StackType_t sharedStack[2048];
  static StaticTask_t sharedTCB;

  TaskHandle_t processTaskHandle = NULL;
  QueueHandle_t displayQueue;

  void showMenu();
  void showProcessScreen(GPSU::ProcessType type);
  void updateTrafficLightDisplay(int state);
  void updateWaterLevelDisplay(const int state, int level);
  void sendDisplayCommand(const DisplayCommand &cmd);
  void run_process(GPSU::ProcessType type);
  void startProcess(GPSU::ProcessType type);

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
  static void water_level_task(void *param);

  static const MenuItem menuItems[];
  static constexpr size_t MENU_ITEM_COUNT = 6;

  // helpers
  void drawAlignText(AlignMent align, const char *text, uint8_t font,
                     Colors txt = Colors::main, Colors bg = Colors::black);
  std::tuple<int16_t, int16_t>
  calculateAlignment(AlignMent align, int16_t Width, int16_t Height);

  // void setUpSprites(GPSU::ProcessType type);
  void processScreenSetup();
  void processScreenExecute();
};

} // namespace GUI

#endif // GUI_HPP
