#ifndef GUI_CONSTANTS_HPP
#define GUI_CONSTANTS_HPP
#include "stdint.h"
namespace GUI {
static constexpr uint8_t MAX_WIDTH = 135;
static constexpr uint8_t MAX_HEIGHT = 240;
static constexpr uint8_t TOP_MARGIN_PX = 5;
static constexpr uint8_t BOTTOM_MARGIN_PX = 5;
static constexpr uint8_t LEFT_MARGIN_PX = 5;
static constexpr uint8_t RIGHT_MARGIN_PX = 5;
static constexpr uint8_t PADDING_PX = 5;
static constexpr uint8_t IMAGE_TOP_PX = 50;
static constexpr uint8_t FRAME_TOP_PX = 30;

static constexpr uint8_t IMG_WIDTH = 130;
static constexpr uint8_t IMG_HEIGHT = 144;

static constexpr uint8_t FRMAE_WIDTH = 133;
static constexpr uint8_t FRAME_HEIGHT = 164;

static constexpr uint8_t TANK_RADIUS = 5;
static constexpr uint8_t TANK_BORDER_THK = 5;
static constexpr uint8_t TANK_WIDTH = IMG_WIDTH - 2 * TANK_BORDER_THK;
static constexpr uint8_t TANK_HEIGHT = IMG_HEIGHT - 2 * TANK_BORDER_THK;

static constexpr uint8_t MENU_FONT = 2;
static constexpr uint8_t MENU_VERTICAL_PADDING = 10;
enum class Colors : uint16_t {
  black = 0x0000,
  white = 0xFFFF,
  main = 0xE6FD,
  menu = 0x30BA,
  logo = 0xB990,
  cursor = 0x30BA,
  frame = 0x30BA,
  process_bg = 0x30BA

};
enum class ScreenRotation : uint8_t { VERTICAL = 0, HORIZONTAl = 1 };
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