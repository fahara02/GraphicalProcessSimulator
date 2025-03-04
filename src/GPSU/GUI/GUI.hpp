#ifndef GUI_HPP
#define GUI_HPP

#include "GUIConstants.hpp"
#include "MenuSelector.hpp"
#include "Process/ProcessDefines.hpp"
#include <TFT_eSPI.h>
#include <memory>
#include <tuple>

namespace GUI {

class Display {
public:
  static Display &getInstance();

  Display(const Display &) = delete;
  Display &operator=(const Display &) = delete;

  TFT_eSPI &Canvas();
  TFT_eSprite &Sprite();

  void show_menu();

  static void processTrafficLight();
  static void processWaterLevel();
  static void processStepperMotorControl();
  static void processStateMachine();
  static void processObjectCounter();
  static void processMotorControl();

  void init();

private:
  // Private constructor.
  explicit Display();

  void run_process(GPSU::ProcessType type);

  static void onSelectionChanged(size_t index);
  static void onItemSelected(size_t index);

  bool initialised;
  std::unique_ptr<TFT_eSPI> canvas_;
  std::unique_ptr<TFT_eSprite> sprite_;
  std::unique_ptr<MenuSelector> menu_;
  GPSU::ProcessType current_process_;

  static const MenuItem menuItems[];
  static constexpr size_t MENU_ITEM_COUNT = 6;

  // helpers
  void drawAlignText(AlignMent align, const char *text, uint8_t font,
                     Colors txt = Colors::main, Colors bg = Colors::none);
  std::tuple<int16_t, int16_t>
  calculateAlignment(AlignMent align, int16_t Width, int16_t Height);
};

} // namespace GUI

#endif // GUI_HPP
