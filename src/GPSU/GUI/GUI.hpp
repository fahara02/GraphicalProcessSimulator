#ifndef GUI_HPP
#define GUI_HPP

#include "Assets/AssetImages.hpp"
#include "GUIConstants.hpp"
#include "Gradient.hpp"
#include "MenuSelector.hpp"
#include "Process/ProcessDefines.hpp"
#include "StateMachines/StateDefines.hpp"
#include "atomic"
#include <TFT_eSPI.h>
#include <memory>
#include <tuple>

namespace GUI {

enum class CmdType {
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

struct Command {
  CmdType type;
  GPSU::ProcessType process_type;
  struct {
    size_t index;
    size_t items;
    const char *label;
  } cursor;

  struct {
    TrafficLight::Context tl_context;
    WaterLevel::Context wl_context;
    StepperMotor::Context st_context;
    ObjectCounter::Context oc_context;
  } contexts;

  uint16_t analog_ch0;
  uint16_t analog_ch1;
};
class Display {
public:
  static Display &getInstance(const MenuItem *items, size_t count);

  TFT_eSPI &Canvas();
  TFT_eSprite &Sprite();

  void init();
  void sendDisplayCommand(const Command &cmd);
  void prepare_assets(GPSU::ProcessType type);
  void setMenuItems(const MenuItem *items, size_t count);
  void setCursorIndex(size_t index);

  Display(const Display &) = delete;
  Display &operator=(const Display &) = delete;
  void resetQueue() { xQueueReset(displayQueue); }

private:
  // Private constructor.
  explicit Display(const MenuItem *items, size_t count);
  const MenuItem *menuItems_ = nullptr;
  size_t menuItemCount_ = 0;
  uint8_t rotation_ = 0;
  bool initialised;

  bool setup_traffic = false;
  bool setup_waterlevel = false;
  bool setup_stepper = false;
  bool setup_counter = false;

  std::unique_ptr<TFT_eSPI> canvas_;
  std::unique_ptr<TFT_eSprite> bg_;
  std::unique_ptr<TFT_eSprite> label_;
  std::unique_ptr<TFT_eSprite> layer_1;
  std::unique_ptr<TFT_eSprite> layer_2;
  GPSU::ProcessType current_process_;
  QueueHandle_t displayQueue;
  size_t cursorIndex_ = 0;
  void showMenu();
  void showProcessScreen(GPSU::ProcessType type);
  void updateTrafficLight(Command cmd);
  void updateWaterLevelDisplay(const int state, int level);
  void updateStepperDisplay(const int dir, int step);
  void updateObjectCounter(Command cmd);

  static void onSelectionChanged(size_t index);
  static void onItemSelected(size_t index);
  static void display_loop(void *param);

  void deleteSprites();
  void createSprites();
  void deinitProcessFlags();
  void reset_rotation();
  void change_to_horizontal();

  // helpers
  void drawAlignText(AlignMent align, const char *text, uint8_t font,
                     Colors txt = Colors::main, Colors bg = Colors::black);
  std::tuple<int16_t, int16_t>
  calculateAlignment(AlignMent align, int16_t Width, int16_t Height);

  // void setUpSprites(GPSU::ProcessType type);
  void processScreenSetup();
  void processScreenExecute(int angle = 0);

  // gui helpers
  struct drawData {
    int32_t x;
    int32_t y;
    int32_t w;
    int32_t h;
    int id;
  };
  void drawBox(TFT_eSprite *sprite, drawData &data, Items::State state);
};

} // namespace GUI

#endif // GUI_HPP

// int object_counter_state; //  (e.g., 0=empty, 1=incr, 2=decr ,3=error)
// int state_machine_state;  //  (e.g., 0=init, 1=first, 2=second ,3=end)
// int motor_control_state;  //  (e.g., 0=run, 1=stop, 2=fwd ,3=rev)
