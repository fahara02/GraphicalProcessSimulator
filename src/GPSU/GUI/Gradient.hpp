#pragma once
#include "TFT_eSPI.h"
#include <array>
#include <cstdint>

namespace Gradient {

// Generic gradient generator
template <size_t Steps, typename ColorFn>
constexpr auto generateGradient(ColorFn colorFunc) {
  std::array<uint16_t, Steps> gradient{};
  for (size_t i = 0; i < Steps; ++i) {
    gradient[i] = colorFunc(i, Steps);
  }
  return gradient;
}

// Define color565 function
constexpr uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((static_cast<uint16_t>(r & 0xF8) << 8) |
          (static_cast<uint16_t>(g & 0xFC) << 3) | (b >> 3));
}
// Color function for linear grayscale gradient
inline auto grayscaleColorFunc = [](size_t i, size_t steps) {
  uint8_t intensity = static_cast<uint8_t>((i * 255) / (steps - 1));
  return color565(intensity, intensity, intensity);
};

inline auto redToBlueColorFunc = [](size_t i, size_t steps) {
  uint8_t red = static_cast<uint8_t>(255 - (255 * i) / (steps - 1));
  uint8_t blue = static_cast<uint8_t>((255 * i) / (steps - 1));
  uint8_t green = 0;
  return color565(red, green, blue);
};

// Generic color interpolator between two colors
template <uint16_t StartColor, uint16_t EndColor> struct ColorInterpolator {
  constexpr uint16_t operator()(size_t i, size_t steps) const {

    size_t validated_steps = (steps < 2) ? 2 : steps;

    // Unpack start color components
    constexpr uint8_t startR = (StartColor >> 11) << 3;
    constexpr uint8_t startG = ((StartColor >> 5) & 0x3F) << 2;
    constexpr uint8_t startB = (StartColor & 0x1F) << 3;

    // Unpack end color components
    constexpr uint8_t endR = (EndColor >> 11) << 3;
    constexpr uint8_t endG = ((EndColor >> 5) & 0x3F) << 2;
    constexpr uint8_t endB = (EndColor & 0x1F) << 3;

    // Use signed arithmetic to handle negative differences
    long diffR = static_cast<long>(endR) - static_cast<long>(startR);
    long diffG = static_cast<long>(endG) - static_cast<long>(startG);
    long diffB = static_cast<long>(endB) - static_cast<long>(startB);

    long step = static_cast<long>(i);
    long total_steps = static_cast<long>(validated_steps - 1);

    // Calculate interpolated values with rounding
    long r = static_cast<long>(startR) +
             (diffR * step + total_steps / 2) / total_steps;
    long g = static_cast<long>(startG) +
             (diffG * step + total_steps / 2) / total_steps;
    long b = static_cast<long>(startB) +
             (diffB * step + total_steps / 2) / total_steps;

    // Clamp to 0-255
    r = (r < 0) ? 0 : (r > 255) ? 255 : r;
    g = (g < 0) ? 0 : (g > 255) ? 255 : g;
    b = (b < 0) ? 0 : (b > 255) ? 255 : b;

    return color565(static_cast<uint8_t>(r), static_cast<uint8_t>(g),
                    static_cast<uint8_t>(b));
  }
};

// Symmetric gradient color function as a struct
struct SymmetricGradientColorFunc {
  constexpr uint16_t operator()(size_t i, size_t steps) const {
    const uint8_t minIntensity = 25;
    const uint8_t maxIntensity = 245;
    size_t half = steps / 2;
    uint8_t intensity;
    if (half == 0) {
      intensity = minIntensity; // Avoid division by zero
    } else if (i <= half) {
      intensity =
          minIntensity +
          static_cast<uint8_t>(
              (static_cast<long>(maxIntensity - minIntensity) * i) / half);
    } else {
      intensity =
          minIntensity +
          static_cast<uint8_t>((static_cast<long>(maxIntensity - minIntensity) *
                                (steps - 1 - i)) /
                               half);
    }
    return color565(intensity, intensity, intensity);
  }
};

// Example color definitions
constexpr uint16_t BLACK = 0x0000;
constexpr uint16_t WHITE = color565(255, 255, 255);
constexpr uint16_t RED = color565(255, 0, 0);
constexpr uint16_t BLUE = color565(0, 0, 255);
constexpr uint16_t ITEM_BG_START = color565(160, 160, 160);   // Light grey
constexpr uint16_t ITEM_BG_END = color565(50, 200, 255);      // Vibrant cyan
constexpr uint16_t ITEM_SELECTED_START = color565(0, 0, 128); // Navy
constexpr uint16_t ITEM_SELECTED_END = color565(0, 255, 255); // Bright cyan

// Generate gradients
constexpr auto itemBgGradient =
    generateGradient<50>(ColorInterpolator<ITEM_BG_START, ITEM_BG_END>{});
constexpr auto itemSelectedBgGradient = generateGradient<50>(
    ColorInterpolator<ITEM_SELECTED_START, ITEM_SELECTED_END>{});

// Generate gradients using the interpolator
constexpr auto grays = generateGradient<50>(ColorInterpolator<BLACK, WHITE>{});
constexpr auto redToBlue = generateGradient<50>(ColorInterpolator<RED, BLUE>{});
constexpr auto lines = generateGradient<50>(SymmetricGradientColorFunc{});
template <size_t Steps>
static inline void
drawCustomGradient(TFT_eSPI *tft, int x, int y, int w, int h,
                   const std::array<uint16_t, Steps> &gradient) {
  float step = static_cast<float>(w) / gradient.size();
  for (size_t i = 0; i < gradient.size(); ++i) {
    tft->fillRect(x + i * step, y, ceil(step), h, gradient[i]);
  }
}

// Usage

} // namespace Gradient