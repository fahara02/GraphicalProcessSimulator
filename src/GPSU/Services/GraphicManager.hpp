#ifndef SERVICES_HPP
#define SERVICES_HPP

#include <TFT_eSPI.h>
#include <memory>

namespace Service {
class Display {
public:
  static Display &getInstance() {
    static Display instance; // Corrected instance declaration
    return instance;
  }

  Display(const Display &) = delete;
  Display &operator=(const Display &) = delete;

  TFT_eSPI &TFT() { return *tft_; }
  TFT_eSprite &Sprite() { return *sprite_; }

private:
  explicit Display()
      : tft_(std::make_unique<TFT_eSPI>()),
        sprite_(std::make_unique<TFT_eSprite>(tft_.get())) {}

  std::unique_ptr<TFT_eSPI> tft_;
  std::unique_ptr<TFT_eSprite> sprite_;
};
} // namespace Service

#endif
