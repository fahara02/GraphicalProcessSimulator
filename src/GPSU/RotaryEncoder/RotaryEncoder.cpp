#include "RotaryEncoder.hpp"
#include "esp_log.h"
#define LOG_TAG "RotaryEncoder"
using namespace COMPONENT;
Encoder::Encoder(uint8_t steps, gpio_num_t aPin, gpio_num_t bPin,
                 gpio_num_t buttonPin, bool encoderPinPulledDown)
    : aPin_(aPin), bPin_(bPin), buttonPin_(buttonPin), encoderSteps_(steps),
      encoderPinPulledDown_(encoderPinPulledDown), isEnabled_(true) {}

void Encoder::begin() {
  gpio_config_t ioConfig = {};
  ioConfig.intr_type =
      GPIO_INTR_ANYEDGE; // Enable interrupts for CHANGE on encoder pins
  ioConfig.mode = GPIO_MODE_INPUT;

  // Configure encoder pins
  ioConfig.pin_bit_mask = (1ULL << aPin_) | (1ULL << bPin_);
  ioConfig.pull_down_en =
      encoderPinPulledDown_ ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
  ioConfig.pull_up_en =
      encoderPinPulledDown_ ? GPIO_PULLUP_DISABLE : GPIO_PULLUP_ENABLE;
  gpio_config(&ioConfig);

  // Configure button pin if defined
  if (buttonPin_ != GPIO_NUM_NC) {
    ioConfig.pin_bit_mask = (1ULL << buttonPin_);
    ioConfig.intr_type =
        GPIO_INTR_POSEDGE; // Enable interrupts for RISING edge on button pin
    ioConfig.pull_down_en =
        isButtonPulldown_ ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
    ioConfig.pull_up_en =
        isButtonPulldown_ ? GPIO_PULLUP_DISABLE : GPIO_PULLUP_ENABLE;
    gpio_config(&ioConfig);
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

  encoderQueue = xQueueCreate(QUEUE_SIZE, sizeof(int8_t));
  buttonQueue = xQueueCreate(QUEUE_SIZE, sizeof(bool));

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

      if (encoder->encoderCallback_) {
        encoder->encoderCallback_(); // Call callback if set
      }

      // Update position
      long prevPosition =
          encoder->encoderPosition_.load() / encoder->encoderSteps_;
      encoder->encoderPosition_ += direction;
      long newPosition =
          encoder->encoderPosition_.load() / encoder->encoderSteps_;

      if (newPosition != prevPosition && encoder->rotaryAccelerationCoef_ > 1) {
        constexpr unsigned long ACCELERATION_LONG_CUTOFF = 200;
        constexpr unsigned long ACCELERATION_SHORT_CUTOFF = 4;

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

void Encoder::ButtonMonitorTask(void *param) {
  Encoder *encoder = static_cast<Encoder *>(param);

  static std::atomic<unsigned long> lastDebounceTime{0};
  constexpr unsigned long DEBOUNCE_DELAY = 50;

  while (true) {
    bool buttonPressed;
    if (xQueueReceive(encoder->buttonQueue, &buttonPressed, portMAX_DELAY)) {
      unsigned long currentTime = millis();

      if (!encoder->isEnabled_) {
        encoder->buttonState_.store(ButtonState::BTN_DISABLED);
        continue;
      }

      if (currentTime - lastDebounceTime.load() < DEBOUNCE_DELAY) {
        continue;
      }

      ButtonState currentState = encoder->buttonState_.load();
      if (buttonPressed && currentState == ButtonState::UP) {
        encoder->buttonState_.store(ButtonState::PUSHED);
        lastDebounceTime.store(currentTime);
      } else if (!buttonPressed && currentState == ButtonState::DOWN) {
        encoder->buttonState_.store(ButtonState::RELEASED);
        lastDebounceTime.store(currentTime);
      } else {
        encoder->buttonState_.store(buttonPressed ? ButtonState::DOWN
                                                  : ButtonState::UP);
      }

      if (encoder->buttonCallback_) {
        encoder->buttonCallback_();
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
  static bool wasTimeouted = false;

  bool buttonPressed = !gpio_get_level(buttonPin_);

  switch (state) {
  case ButtonClickState::IDLE:
    if (buttonPressed) {
      // Start debounce timer
      waitStartTime = millis();
      state = ButtonClickState::WAIT_FOR_RELEASE;
    }
    break;

  case ButtonClickState::WAIT_FOR_RELEASE:
    if (!buttonPressed) {
      // Button released within debounce time
      if (millis() - waitStartTime > 30) {
        wasTimeouted = false;
        state = ButtonClickState::IDLE;
        return true;
      } else {
        // Too quick, ignore
        state = ButtonClickState::IDLE;
      }
    } else if (millis() - waitStartTime > maximumWaitMilliseconds) {
      // Timeout while waiting for release
      wasTimeouted = true;
      state = ButtonClickState::IDLE;
    }
    break;

  case ButtonClickState::WAIT_FOR_TIMEOUT:
    if (!buttonPressed) {
      state = ButtonClickState::IDLE;
    } else if (millis() - waitStartTime > maximumWaitMilliseconds) {
      wasTimeouted = true;
      state = ButtonClickState::IDLE;
    }
    break;
  }

  return false;
}
