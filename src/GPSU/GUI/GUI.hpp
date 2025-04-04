#ifndef GUI_HPP
#define GUI_HPP

#include "Assets/AssetImages.hpp"
#include "Assets/NotoSansMonoSCB20.h"
#include "Assets/orbitron18.h"
#include "GUIConstants.hpp"
#include "Gradient.hpp"
#include "MenuSelector.hpp"
#include "Process/ProcessDefines.hpp"
#include "StateMachines/StateDefines.hpp"
#include "atomic"
#include <TFT_eSPI.h>
#include <algorithm>
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
  void updateCursorIndex(size_t index);

  Display(const Display &) = delete;
  Display &operator=(const Display &) = delete;
  void resetQueue() { xQueueReset(displayQueue); }
  ScreenRotation getScreenRotation() const {
    return static_cast<ScreenRotation>(rotation_);
  }

private:
  // Private constructor.
  explicit Display(const MenuItem *items, size_t count);
  const MenuItem *menuItems_ = nullptr;
  size_t menuItemCount_ = 0;
  uint8_t rotation_ = 0;
  uint8_t visible_items = 3;
  bool initialised;
  bool needsRedraw = true;
  int16_t scrollOffset = 0;
  uint32_t scrollDebounce = 0;
  struct {
    int16_t textWidth;
    int16_t textHeight;
    int16_t padding;
    int16_t radius;
  } menuMetrics;

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

  void calculateMenuMetrics() {
    const ScreenRotation rotation = getScreenRotation();
    menuMetrics.padding = PADDING_PX;
    bg_->setFreeFont(&FreeSans12pt7b);
    menuMetrics.textHeight = bg_->fontHeight();

    // Calculate available vertical space
    const int16_t startY = 50;
    const int16_t availableHeight = canvas_->height() - startY - 20;

    // Adjust visible items based on available height and text height
    visible_items = availableHeight / menuMetrics.textHeight;

    // Additional adjustments for vertical padding if needed
    if (rotation == ScreenRotation::VERTICAL) {
      menuMetrics.textHeight += MENU_VERTICAL_PADDING;
    }

    menuMetrics.radius = 8;
  }
  void rotateMenu() {
    rotation_ = (rotation_ + 1) % 4;
    canvas_->setRotation(rotation_);
    deleteSprites();
    createSprites();
    calculateMenuMetrics();
    needsRedraw = true;
  }
  void drawMenuFramework() {
    bg_->fillScreen(Colors::black);

    // Draw menu background with vibrant purple
    bg_->fillRoundRect(10, 10, bg_->width(), bg_->height() - 20,
                       menuMetrics.radius, Colors::menu_bg);
    bg_->drawRoundRect(10, 10, bg_->width() - 20, bg_->height() - 20,
                       menuMetrics.radius, Colors::frame);

    // Title with white text
    const char *title = "Main Menu";
    bg_->setFreeFont(&Orbitron_Medium_18);
    bg_->setTextColor(Colors::title_text, Colors::menu_bg);
    bg_->drawString(title, (bg_->width() - bg_->textWidth(title)) / 2, 15);
  }
  void drawMenuItem(int16_t yPos, size_t itemIndex) {
    const bool isSelected = (itemIndex == cursorIndex_);
    const int16_t itemWidth = bg_->width() - 40;
    char truncatedLabel[32];

    bg_->setFreeFont(&FreeSans12pt7b);
    bg_->setTextSize(1);

    truncateText(menuItems_[itemIndex].label, truncatedLabel, itemWidth - 20);

    int16_t textX = 20 + (itemWidth - bg_->textWidth(truncatedLabel)) / 2;
    int16_t textY = yPos + (menuMetrics.textHeight - bg_->fontHeight()) / 2;

    // Vibrant orange/magenta backgrounds
    bg_->fillRoundRect(20, yPos, itemWidth, menuMetrics.textHeight,
                       menuMetrics.radius,
                       isSelected ? Colors::item_selected_bg : Colors::item_bg);

    // High-contrast text colors
    bg_->setTextColor(isSelected ? Colors::white : Colors::black);
    bg_->drawString(truncatedLabel, textX, textY);

    if (isSelected) {
      bg_->drawRoundRect(20, yPos, itemWidth, menuMetrics.textHeight,
                         menuMetrics.radius, Colors::cursor); // Yellow border
    }
  }

  void drawMenuItems() {
    const int16_t startY = 50;
    const int16_t visibleHeight = visible_items * menuMetrics.textHeight;

    // Draw scroll indicators if needed
    if (scrollOffset > 0) {
      drawTriangle(bg_->width() / 2, startY - 15, true);
    }
    if (scrollOffset < menuItemCount_ - visible_items) {
      drawTriangle(bg_->width() / 2, startY + visibleHeight + 5, false);
    }

    // Draw each visible item
    for (uint8_t i = 0; i < visible_items; ++i) {
      const size_t itemIndex = scrollOffset + i;
      if (itemIndex >= menuItemCount_)
        break;
      const int16_t yPos = startY + i * menuMetrics.textHeight;
      drawMenuItem(yPos, itemIndex);
    }
  }

  void drawTriangle(int16_t x, int16_t y, bool up) {
    const uint8_t size = 8;
    bg_->fillTriangle(x - size, y + (up ? size : 0), x + size,
                      y + (up ? size : 0), x, y + (up ? 0 : size),
                      Colors::scroll_indicator);
  }

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
  void truncateText(const char *input, char *output, int maxWidth) {
    strncpy(output, input, 31);
    output[31] = '\0';
    bg_->setFreeFont(&FreeSans12pt7b); // Ensure correct font is set
    while (bg_->textWidth(output) > maxWidth && strlen(output) > 3) {
      output[strlen(output) - 1] = '\0';
      output[strlen(output) - 1] = '.';
      output[strlen(output) - 2] = '.';
    }
  }
  // Helpers for visual depth
  uint16_t lightenColor(uint16_t color) {
    return ((color & 0xF79E) >> 1) + 0x4210; // Lighten by ~20%
  }

  uint16_t darkenColor(uint16_t color) {
    return (color & 0xF79E) >> 1; // Darken by ~20%
  }

  uint16_t interpolateColor(uint16_t c1, uint16_t c2, uint8_t ratio) {
    return ((c1 & 0xF81F) * (255 - ratio) + (c2 & 0xF81F) * ratio) >> 8;
  }
  uint16_t swapBytes(uint16_t color) { return (color >> 8) | (color << 8); }
};

} // namespace GUI

#endif // GUI_HPP

// int object_counter_state; //  (e.g., 0=empty, 1=incr, 2=decr ,3=error)
// int state_machine_state;  //  (e.g., 0=init, 1=first, 2=second ,3=end)
// int motor_control_state;  //  (e.g., 0=run, 1=stop, 2=fwd ,3=rev)
