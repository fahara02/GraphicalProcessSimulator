#include "RotaryEncoder.hpp"
#include "esp_log.h"
#define LOG_TAG "RotaryEncoder"
using namespace COMPONENT;
Encoder::Encoder(uint8_t steps, gpio_num_t aPin, gpio_num_t bPin,
                 gpio_num_t buttonPin, bool encoderPinPulledDown)
    : aPin_(aPin), bPin_(bPin), buttonPin_(buttonPin), encoderSteps_(steps),
      encoderPinPulledDown_(encoderPinPulledDown), isEnabled_(true) {}

void Encoder::configurePin(gpio_num_t pin, gpio_int_type_t intrType,
                           bool pullDown, bool pullUp) {
  gpio_config_t ioConfig = {};
  ioConfig.intr_type = intrType;
  ioConfig.mode = GPIO_MODE_INPUT;
  ioConfig.pin_bit_mask = (1ULL << pin);
  ioConfig.pull_down_en =
      pullDown ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
  ioConfig.pull_up_en = pullUp ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
  gpio_config(&ioConfig);
}

void Encoder::begin() {
  configurePin(aPin_, GPIO_INTR_ANYEDGE, encoderPinPulledDown_,
               !encoderPinPulledDown_);
  configurePin(bPin_, GPIO_INTR_ANYEDGE, encoderPinPulledDown_,
               !encoderPinPulledDown_);
  if (buttonPin_ != GPIO_NUM_NC) {
    configurePin(buttonPin_, GPIO_INTR_POSEDGE, isButtonPulldown_,
                 !isButtonPulldown_);
  }

  // Attach interrupts
  gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
  gpio_isr_handler_add(
      aPin_, [](void *arg) { static_cast<Encoder *>(arg)->encoderISR(); },
      this);
  gpio_isr_handler_add(
      bPin_, [](void *arg) { static_cast<Encoder *>(arg)->encoderISR(); },
      this);
  if (buttonPin_ != GPIO_NUM_NC) {
    gpio_isr_handler_add(
        buttonPin_, [](void *arg) { static_cast<Encoder *>(arg)->buttonISR(); },
        this);
  }

  encoderQueue = xQueueCreateStatic(QUEUE_SIZE, sizeof(int8_t),
                                    encoderQueueBuffer, &encoderQueueStorage);
  buttonQueue = xQueueCreateStatic(QUEUE_SIZE, sizeof(int8_t),
                                   buttonQueueBuffer, &buttonQueueStorage);

  xTaskCreatePinnedToCore(
      EncoderMonitorTask, "EncoderMonitor", GPSU_CORE::EncoderTaskStack, this,
      GPSU_CORE::EncoderTask_Priority, nullptr, GPSU_CORE::EncoderTask_CORE);
  if (buttonPin_ != GPIO_NUM_NC) {
    xTaskCreatePinnedToCore(
        ButtonMonitorTask, "ButtonMonitor", GPSU_CORE::BtnTaskStack, this,
        GPSU_CORE::BtnTask_Priority, nullptr, GPSU_CORE::BtnTask_CORE);
  }
}
void Encoder::setup(void (*ISR_callback)(void)) {
  encoderCallback_ = ISR_callback;
}

void Encoder::setup(void (*ISR_callback)(void), void (*ISR_button)(void)) {
  encoderCallback_ = ISR_callback;
  buttonCallback_ = ISR_button;
}
void IRAM_ATTR Encoder::onTimer(void *arg) {
  timeCounter.fetch_add(1,
                        std::memory_order_relaxed); // Increment counter safely
}
void Encoder::setupTimer() {
  timer_config_t config = {.alarm_en = TIMER_ALARM_EN,
                           .counter_en = TIMER_PAUSE,
                           .intr_type = TIMER_INTR_LEVEL,
                           .counter_dir = TIMER_COUNT_UP,
                           .auto_reload = TIMER_AUTORELOAD_EN,
                           .divider = TIMER_DIVIDER};
  timer_init(TIMER_GROUP_0, TIMER_0, &config);

  timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
  timer_set_alarm_value(TIMER_GROUP_0, TIMER_0,
                        TIMER_SCALE * TIMER_INTERVAL_MS / 1000);
  timer_enable_intr(TIMER_GROUP_0, TIMER_0);
  timer_isr_register(TIMER_GROUP_0, TIMER_0, Encoder::onTimer, nullptr,
                     ESP_INTR_FLAG_IRAM, nullptr);

  timer_start(TIMER_GROUP_0, TIMER_0);
}

void IRAM_ATTR Encoder::encoderISR() {
  if (!isEnabled_) {

    return;
  }

  oldAB_ <<= 2;
  uint8_t state = (gpio_get_level(bPin_) << 1) | gpio_get_level(aPin_);
  oldAB_ |= (state & 0x03);
  int8_t currentDirection = encoderStates_[oldAB_ & 0x0F];

  if (currentDirection != 0) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(encoderQueue, &currentDirection,
                      &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
};
void IRAM_ATTR Encoder::buttonISR() {
  bool buttonPressed = !gpio_get_level(buttonPin_);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(buttonQueue, &buttonPressed, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
void Encoder::EncoderMonitorTask(void *param) {
  Encoder *encoder = static_cast<Encoder *>(param); // Cast param
  int8_t direction;

  while (true) {

    if (xQueueReceive(encoder->encoderQueue, &direction, portMAX_DELAY)) {
      unsigned long now = millis();

      if (!encoder->isEnabled_) {
        continue;
      }

      // Update position
      long prevPosition =
          encoder->encoderPosition_.load() / encoder->encoderSteps_;
      encoder->encoderPosition_ += direction;
      long newPosition =
          encoder->encoderPosition_.load() / encoder->encoderSteps_;

      if (newPosition != prevPosition && encoder->rotaryAccelerationCoef_ > 1) {

        int8_t lastDirection = encoder->lastMovementDirection_.load();
        if (direction == lastDirection && direction != 0 &&
            lastDirection != 0) {
          unsigned long timeSinceLastMotion =
              now - encoder->lastMovementTime_.load();
          if (timeSinceLastMotion < ACCELERATION_LONG_CUTOFF) {
            unsigned long limitedTime =
                std::max(ACCELERATION_SHORT_CUTOFF, timeSinceLastMotion);
            int adjustment = encoder->rotaryAccelerationCoef_ / limitedTime;
            encoder->encoderPosition_ +=
                direction > 0 ? adjustment : -adjustment;
          }
        }

        encoder->lastMovementTime_.store(now);
        encoder->lastMovementDirection_.store(direction);
        if (encoder->encoderCallback_) {
          encoder->encoderCallback_();
        }
      }

      long adjustedPosition =
          encoder->encoderPosition_.load() / encoder->encoderSteps_;
      int maxPosition = encoder->maxEncoderValue_ / encoder->encoderSteps_;
      int minPosition = encoder->minEncoderValue_ / encoder->encoderSteps_;

      if (adjustedPosition > maxPosition) {
        if (encoder->circleValues_) {
          long delta = adjustedPosition - maxPosition;
          encoder->encoderPosition_.store(encoder->minEncoderValue_ +
                                          (delta * encoder->encoderSteps_));
        } else {
          encoder->encoderPosition_.store(encoder->maxEncoderValue_);
        }
      } else if (adjustedPosition < minPosition) {
        if (encoder->circleValues_) {
          long delta = adjustedPosition - minPosition;
          encoder->encoderPosition_.store(encoder->maxEncoderValue_ +
                                          (delta * encoder->encoderSteps_));
        } else {
          encoder->encoderPosition_.store(encoder->minEncoderValue_);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  vTaskDelete(NULL);
}

bool Encoder::debounce(bool currentState, unsigned long &lastTime,
                       unsigned long delay) {
  unsigned long now = timeCounter.load(std::memory_order_relaxed);
  if (now - lastTime > delay) {
    lastTime = now;
    return currentState;
  }
  return false;
}

void Encoder::ButtonMonitorTask(void *param) {
  Encoder *encoder = static_cast<Encoder *>(param);

  unsigned long lastDebounceTime = 0;

  while (true) {
    bool buttonPressed;
    if (xQueueReceive(encoder->buttonQueue, &buttonPressed, portMAX_DELAY)) {
      unsigned long currentTime = timeCounter.load(std::memory_order_relaxed);

      if (!encoder->isEnabled_) {
        encoder->buttonState_.store(ButtonState::BTN_DISABLED);
        continue;
      }
      if (encoder->debounce(buttonPressed, lastDebounceTime, DEBOUNCE_DELAY)) {
        ButtonState currentState = encoder->buttonState_.load();

        if (buttonPressed && currentState == ButtonState::UP) {
          encoder->buttonState_.store(ButtonState::PUSHED);
        } else if (!buttonPressed && currentState == ButtonState::DOWN) {
          encoder->buttonState_.store(ButtonState::RELEASED);
        } else {
          encoder->buttonState_.store(buttonPressed ? ButtonState::DOWN
                                                    : ButtonState::UP);
        }

        if (encoder->buttonCallback_) {
          encoder->buttonCallback_();
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }

  vTaskDelete(NULL);
}

void Encoder::setBoundaries(long minValue, long maxValue, bool circleValues) {
  minEncoderValue_ = minValue * encoderSteps_;
  maxEncoderValue_ = maxValue * encoderSteps_;
  circleValues_ = circleValues;
}
long Encoder::readEncoder() const {
  long position = encoderPosition_ / encoderSteps_;
  if (position > maxEncoderValue_ / encoderSteps_)
    return maxEncoderValue_ / encoderSteps_;
  if (position < minEncoderValue_ / encoderSteps_)
    return minEncoderValue_ / encoderSteps_;
  return position;
}
void Encoder::reset(long newValue) {

  long adjustedValue = newValue * encoderSteps_ + correctionOffset_;
  encoderPosition_ = adjustedValue;

  if (encoderPosition_ > maxEncoderValue_) {
    encoderPosition_ = circleValues_ ? minEncoderValue_ : maxEncoderValue_;
  }
  if (encoderPosition_ < minEncoderValue_) {
    encoderPosition_ = circleValues_ ? maxEncoderValue_ : minEncoderValue_;
  }

  lastReadEncoderPosition_ = encoderPosition_ / encoderSteps_;
}

bool Encoder::isEncoderButtonDown() {
  return gpio_get_level(buttonPin_) == 0 ? false : true;
}

bool Encoder::isEncoderButtonClicked(unsigned long maximumWaitMilliseconds) {
  static enum class ButtonClickState {
    IDLE,
    WAIT_FOR_RELEASE,
    WAIT_FOR_TIMEOUT
  } state = ButtonClickState::IDLE;

  static unsigned long waitStartTime = 0;
  static unsigned long lastDebounceTime = 0; // Debounce timer
  static bool wasTimeouted = false;

  bool buttonPressed = !gpio_get_level(buttonPin_);

  // Use debounce helper to validate button press/release
  if (!debounce(buttonPressed, lastDebounceTime, DEBOUNCE_DELAY)) {
    return false; // Ignore changes within debounce period
  }

  switch (state) {
  case ButtonClickState::IDLE:
    if (buttonPressed) {
      // Start timing when button is first pressed
      waitStartTime = timeCounter.load(std::memory_order_relaxed);
      state = ButtonClickState::WAIT_FOR_RELEASE;
    }
    break;

  case ButtonClickState::WAIT_FOR_RELEASE:
    if (!buttonPressed) {
      // Button released after debounce period
      if (timeCounter.load(std::memory_order_relaxed) - waitStartTime >
          DEBOUNCE_DELAY) {
        wasTimeouted = false;
        state = ButtonClickState::IDLE;
        return true;
      } else {
        // Release was too quick; ignore
        state = ButtonClickState::IDLE;
      }
    } else if (timeCounter.load(std::memory_order_relaxed) - waitStartTime >
               maximumWaitMilliseconds) {
      // Timeout occurred before button was released
      wasTimeouted = true;
      state = ButtonClickState::IDLE;
    }
    break;

  case ButtonClickState::WAIT_FOR_TIMEOUT:
    if (!buttonPressed) {
      // Reset state when button is released after timeout
      state = ButtonClickState::IDLE;
    } else if (timeCounter.load(std::memory_order_relaxed) - waitStartTime >
               maximumWaitMilliseconds) {
      // Timeout occurred while waiting for release
      wasTimeouted = true;
      state = ButtonClickState::IDLE;
    }
    break;
  }

  return false;
}
