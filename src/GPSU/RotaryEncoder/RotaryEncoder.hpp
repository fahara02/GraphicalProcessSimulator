#ifndef ROTARY_ENCODER_HPP
#define ROTARY_ENCODER_HPP
#include "Arduino.h"
#include "GPSU_Defines.hpp"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "stdint.h"
#include <atomic>
#include <driver/gpio.h>

namespace COMPONENT {

constexpr uint16_t QUEUE_SIZE = 10;
enum class ButtonState {
  DOWN = 0,
  PUSHED = 1,
  UP = 2,
  RELEASED = 3,
  BTN_DISABLED = 99,
};
class Encoder {

private:
  gpio_num_t aPin_;
  gpio_num_t bPin_;
  gpio_num_t buttonPin_;
  uint8_t encoderSteps_;
  int correctionOffset_ = 2;
  bool isButtonPulldown_;
  bool encoderPinPulledDown_;
  bool isEnabled_;

  xQueueHandle encoderQueue = nullptr;
  xQueueHandle buttonQueue = nullptr;

  long minEncoderValue_ = -2147483648;
  long maxEncoderValue_ = 2147483647;
  std::atomic<long> encoderPosition_{0};
  std::atomic<int8_t> lastMovementDirection_{0};
  std::atomic<unsigned long> lastMovementTime_{0};
  long lastReadEncoderPosition_ = 0;
  unsigned long rotaryAccelerationCoef_ = 150;
  bool circleValues_ = false;

  int8_t oldAB_{0};
  const int8_t encoderStates_[16] = {0,  -1, 1, 0, 1, 0, 0,  -1,
                                     -1, 0,  0, 1, 0, 1, -1, 0};
  std::atomic<ButtonState> buttonState_{ButtonState::UP};

  void (*encoderCallback_)() = nullptr;
  void (*buttonCallback_)() = nullptr;

  bool isEncoderButtonClicked(unsigned long maximumWaitMilliseconds = 300);
  bool isEncoderButtonDown();

public:
  Encoder(uint8_t steps, gpio_num_t aPin, gpio_num_t bPin,
          gpio_num_t buttonPin = GPIO_NUM_NC, bool encoderPinPulledDown = true);
  void initGPIOS(){};
  void setBoundaries(long minValue = -100, long maxValue = 100,
                     bool circleValues = false);

  void setup(void (*ISR_callback)(void));
  void setup(void (*ISR_callback)(void), void (*ISR_button)(void));
  void begin();
  void reset(long newValue = 0);
  void enable() { this->isEnabled_ = true; }
  void disable() { this->isEnabled_ = false; }
  long readEncoder() const;
  void setEncoderValue(long newValue) { reset(newValue); };
  long encoderChanged();

  ButtonState readButtonState() { return buttonState_; };
  unsigned long getAcceleration() { return this->rotaryAccelerationCoef_; }
  void setAcceleration(unsigned long acceleration) {
    rotaryAccelerationCoef_ = acceleration;
  }
  void disableAcceleration() { setAcceleration(0); }
  static void EncoderMonitorTask(void *param);
  static void ButtonMonitorTask(void *param);
  void IRAM_ATTR encoderISR();
  void IRAM_ATTR buttonISR();
};

} // namespace COMPONENT

#endif