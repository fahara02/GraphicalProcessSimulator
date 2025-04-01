#ifndef GUI_CONSTANTS_HPP
#define GUI_CONSTANTS_HPP
#include "stdint.h"
namespace GUI {
static constexpr uint8_t MAX_WIDTH = 135;
static constexpr uint8_t MAX_HEIGHT = 240;
static constexpr uint8_t MAX_WIDTH_H = 240;
static constexpr uint8_t MAX_HEIGHT_H = 135;
static constexpr uint8_t TOP_MARGIN_PX = 5;
static constexpr uint8_t BOTTOM_MARGIN_PX = 5;
static constexpr uint8_t LEFT_MARGIN_PX = 5;
static constexpr uint8_t RIGHT_MARGIN_PX = 5;
static constexpr uint8_t PADDING_PX = 5;
static constexpr uint8_t IMAGE_TOP_PX = 50;
static constexpr uint8_t FRAME_TOP_PX = 50;
static constexpr uint8_t IMAGE_TOP_PX_H = 10;
static constexpr uint8_t FRAME_TOP_PX_H = 10;

static constexpr uint8_t LABEL_LEFT_PX = 5;
static constexpr uint8_t LABEL_TOP_PX = 5;

static constexpr uint8_t IMG_WIDTH = 130;
static constexpr uint8_t IMG_HEIGHT = 144;

static constexpr uint8_t IMG_WIDTH_H = 144;
static constexpr uint8_t IMG_HEIGHT_H = 130;
static constexpr uint8_t IMG2_WIDTH = 130;
static constexpr uint8_t IMG2_HEIGHT = 130;

static constexpr uint8_t FRAME_WIDTH = 124;
static constexpr uint8_t FRAME_HEIGHT = 164;

static constexpr uint8_t TANK_RADIUS = 5;
static constexpr uint8_t TANK_BORDER_THK = 5;
static constexpr uint8_t TANK_WIDTH = FRAME_WIDTH - 2 * TANK_BORDER_THK;
static constexpr uint8_t TANK_HEIGHT = FRAME_HEIGHT - 2 * TANK_BORDER_THK;

static constexpr uint8_t MENU_FONT = 4;

static constexpr uint8_t MENU_VERTICAL_PADDING = 2;

struct Colors {
  static constexpr uint16_t black = 0x0000;
  static constexpr uint16_t white = 0xFFFF;
  static constexpr uint16_t main = 0xE6FD;
  static constexpr uint16_t menu = 0x30BA;
  static constexpr uint16_t logo = 0xB990;
  static constexpr uint16_t cursor = 0x30BA;
  static constexpr uint16_t process_bg = 0x30BA;
  static constexpr uint16_t dial_bg = 0x00A3;
  static constexpr uint16_t dial_points = 0x8410;
  static constexpr uint16_t dial_title = 0x15B3;
  static constexpr uint16_t NAVY = 0x000F;
  static constexpr uint16_t DARKGREY = 0x7BEF;
  static constexpr uint16_t SILVER = 0xC618;
  static constexpr uint16_t CYAN = 0x07FF;
  static constexpr uint16_t menu_bg = 0x4A69;          // Dark slate grey
  static constexpr uint16_t item_bg = 0xAD55;          // Light grey
  static constexpr uint16_t item_selected_bg = 0x001F; // Navy blue
  static constexpr uint16_t frame = 0x7BEF;            // Silver
  static constexpr uint16_t title_text = 0xFFFF;       // White
};

enum class ScreenRotation : uint8_t { VERTICAL = 0, HORIZONTAL = 1 };
enum class AlignMent : uint8_t {
  TOP_MIDDLE,
  CENTER_MIDDLE,
  MIDDLE,
  LEFT_X,
  RIGHT_X,
  BOTTOM_MIDDLE
};
} // namespace GUI
#endif