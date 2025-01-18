#ifndef ROTARY_ENCODER_HPP
#define ROTARY_ENCODER_HPP
#include "Arduino.h"
#include "GPSU_Defines.hpp"
#include "driver/timer.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "stdint.h"
#include <atomic>
#include <climits>
#include <driver/gpio.h>
#include <driver/pcnt.h>
#include <soc/pcnt_struct.h>

namespace COMPONENT {

constexpr uint16_t QUEUE_SIZE = 10;
constexpr uint16_t TIMER_DIVIDER = 80;    // 80 MHz clock divided by 80 = 1 MHz
constexpr uint16_t TIMER_INTERVAL_MS = 1; // Interval in milliseconds

#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER) // 1 MHz

static constexpr unsigned long DEBOUNCE_DELAY = 50;
static constexpr unsigned long ENCODER_DEBOUNCE_DELAY = 50;
static constexpr unsigned long ACCELERATION_LONG_CUTOFF = 200;
static constexpr unsigned long ACCELERATION_SHORT_CUTOFF = 4;
enum class EncoderType { SINGLE = 0, HALF = 1, FULL = 2 };
enum class PullType {
  INTERNAL_PULLUP = 0,
  INTERNAL_PULLDOWN = 1,
  EXTERNAL_PULLUP = 2,
  EXTERNAL_PULLDOWN = 3,
  NONE = 4
};
enum class Direction { NOROTATION = 0, CLOCKWISE = 1, COUNTERCLOCKWISE = -1 };
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
  pcnt_unit_t unit;
  uint8_t encoderSteps_;
  int correctionOffset_ = 2;
  PullType encoderPinPull_ = PullType::NONE;
  PullType buttonPinPull_ = PullType::NONE;
  bool isEnabled_;

  TaskHandle_t encoderTaskHandle = nullptr;
  TaskHandle_t buttonTaskHandle = nullptr;

  long minEncoderValue_ = LONG_MIN;
  long maxEncoderValue_ = LONG_MAX;
  int8_t oldAB_{0};
  std::atomic<long> encoderPosition_{0};
  std::atomic<int8_t> lastMovementDirection_{0};
  std::atomic<unsigned long> lastMovementTime_{0};
  static std::atomic<unsigned long> timeCounter;
  long lastReadEncoderPosition_ = 0;
  unsigned long rotaryAccelerationCoef_ = 150;
  bool circleValues_ = false;

  const int8_t encoderStates_[16] = {0,  -1, 1, 0, 1, 0, 0,  -1,
                                     -1, 0,  0, 1, 0, 1, -1, 0};
  std::atomic<ButtonState> buttonState_{ButtonState::UP};

  void (*encoderCallback_)() = nullptr;
  void (*buttonCallback_)() = nullptr;

  bool isEncoderButtonClicked(unsigned long maximumWaitMilliseconds = 300);
  bool isEncoderButtonDown();
  // Helper Functions
  void configurePin(gpio_num_t pin, gpio_int_type_t intrType,
                    PullType pullType);
  bool debounce(bool currentState, unsigned long &lastTime,
                unsigned long delay);
  void initGPIOS();

public:
  Encoder(uint8_t steps, gpio_num_t aPin, gpio_num_t bPin,
          gpio_num_t buttonPin = GPIO_NUM_NC,
          PullType encoderPinPull = PullType::EXTERNAL_PULLUP,
          PullType buttonPinPull = PullType::EXTERNAL_PULLDOWN);

  void setBoundaries(long minValue = -100, long maxValue = 100,
                     bool circleValues = false);

  void setup(void (*ISR_callback)(void));
  void setup(void (*ISR_callback)(void), void (*ISR_button)(void));
  static void setupTimer();
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
  static void IRAM_ATTR onTimer(void *arg);
  void IRAM_ATTR encoderISR();
  void IRAM_ATTR buttonISR();
};

} // namespace COMPONENT

#endif