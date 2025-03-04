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
static constexpr uint8_t PADDING_PX = 2;
enum class Colors {

};
enum class AlignMent : uint8_t {
  TOP_MIDDLE,
  MIDDLE,
  LEFT_X,
  RIGHT_X,
  BOTTOM_MIDDLE

};
} // namespace GUI
#endif