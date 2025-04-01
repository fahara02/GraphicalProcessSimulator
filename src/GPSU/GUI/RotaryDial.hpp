#pragma once

#include "GUIConstants.hpp"
#include <TFT_eSPI.h>
#include <array>
#include <cmath>
namespace GUI {
template <size_t N, size_t P = 60> class RotaryDial {
public:
  RotaryDial(TFT_eSprite &sprite, const std::array<String, N> &labels, int cx,
             int cy, int r)
      : dialSprite(sprite), centerX(cx), centerY(cy), radius(r), cc(labels) {}
  void init() { precomputePositions(); }
  // Set the current angle of the dial
  void setAngle(int newAngle) {
    lastAngle = angle;
    angle = newAngle;
  }

  // Check if the dial needs to be redrawn
  bool needsRedraw() const { return angle != lastAngle; }

  // Draw the dial on the sprite
  void draw() {
    if (!needsRedraw()) {
      return;
    }

    dialSprite.fillSprite(TFT_BLACK);
    dialSprite.fillCircle(centerX, centerY, 124,
                          static_cast<uint16_t>(Colors::dial_bg));

    dialSprite.setTextColor(TFT_WHITE, static_cast<uint16_t>(Colors::dial_bg));
    // dialSprite.setFreeFont(labelFont);

    // Draw labels and lines
    for (size_t i = 0; i < N; i++) {
      int idx = (start_label[i] + angle) % 360;
      dialSprite.drawString(cc[i], x[idx], y[idx]);
      dialSprite.drawLine(px[idx], py[idx], lx[idx], ly[idx], TFT_WHITE);
    }

    // Draw example value and static text
    // dialSprite.setFreeFont(valueFont);
    // dialSprite.drawString("32465", centerX, centerY - 22);
    dialSprite.setTextFont(0);
    dialSprite.drawString("SELECT", centerX, centerY - 42);
    dialSprite.drawString("MENU", 25, 20);
    dialSprite.drawString("MAIN", 25, 6, 2);
    dialSprite.drawString("%", centerX + 44, centerY - 16, 2);

    // Draw points
    for (size_t i = 0; i < P; i++) {
      int idx = (start_points[i] + angle) % 360;
      dialSprite.fillCircle(px[idx], py[idx], 1,
                            static_cast<uint16_t>(Colors::dial_points));
    }

    // Draw indicator triangle
    dialSprite.fillTriangle(centerX - 1, centerY - 70, centerX - 5,
                            centerY - 56, centerX + 4, centerY - 56,
                            TFT_ORANGE);
    lastAngle = angle;
  }

private:
  // Precompute positions for all 360 degrees and starting angles
  void precomputePositions() {
    double rad = 0.01745; // Approximately PI/180
    for (int i = 0; i < 360; i++) {
      x[i] = radius * cos(rad * i) + centerX;
      y[i] = radius * sin(rad * i) + centerY;
      px[i] = (radius - 16) * cos(rad * i) + centerX;
      py[i] = (radius - 16) * sin(rad * i) + centerY;
      lx[i] = (radius - 24) * cos(rad * i) + centerX;
      ly[i] = (radius - 24) * sin(rad * i) + centerY;
    }

    // Calculate starting angles for labels
    for (size_t k = 0; k < N; k++) {
      start_label[k] = static_cast<int>(round(k * 360.0 / N)) % 360;
    }

    // Calculate starting angles for points
    for (size_t k = 0; k < P; k++) {
      start_points[k] = static_cast<int>(round(k * 360.0 / P)) % 360;
    }
  }

  TFT_eSprite &dialSprite;
  int centerX, centerY, radius;
  float x[360], y[360], px[360], py[360], lx[360], ly[360];
  std::array<int, N> start_label;
  std::array<int, P> start_points;
  int angle = 270;
  int lastAngle = 0;
  std::array<String, N> cc;

  // const GFXfont *valueFont = &DSEG7_Modern_Bold_20;
};
} // namespace GUI
