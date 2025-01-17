#ifndef ROTARY_NUMBER_SELECTOR_HPP
#define ROTARY_NUMBER_SELECTOR_HPP
#include "RotaryEncoder.hpp"
#include <memory>
namespace COMPONENT {

class EncoderNumberSelector {
private:
  float minValue = 0;
  float maxValue = 100;
  float step = 2;
  float coeficient = 1;
  std::unique_ptr<Encoder> encoder;

public:
  EncoderNumberSelector(uint8_t steps, gpio_num_t aPin, gpio_num_t bPin,
                        gpio_num_t buttonPin = GPIO_NUM_NC,
                        bool encoderPinPulledDown = true,
                        bool buttonPulledDown = false)
      : encoder(std::make_unique<Encoder>(steps, aPin, bPin, buttonPin,
                                          encoderPinPulledDown,
                                          buttonPulledDown)) {}

  void setRange(float minValue, float maxValue, float step, bool cycleValues,
                unsigned int decimals = 0) {
    this->minValue = minValue;
    this->maxValue = maxValue;
    this->coeficient = pow(10.0, decimals);
    if (maxValue < minValue)
      coeficient *= -1.0;
    this->step = step * this->coeficient;

    long minEncoderValue =
        (long)((this->coeficient * this->minValue) / this->step);
    long maxEncoderValue =
        (long)((this->coeficient * this->maxValue) / this->step);
    long range = maxEncoderValue - minEncoderValue;

    encoder->setBoundaries(minEncoderValue, maxEncoderValue, cycleValues);

    if (range < 20)
      encoder->setAcceleration(0);
    else if (range < 60)
      encoder->setAcceleration(20);
    else if (range < 200)
      encoder->setAcceleration(100);
    else if (range < 1000)
      encoder->setAcceleration(300);
    else
      encoder->setAcceleration(500);
  }
  void setValue(float value) {
    long encoderValue = (long)((coeficient * value) / step);
    encoder->setEncoderValue(encoderValue);
  }

  float getValue() {
    float encoderValue = 1.0 * encoder->readEncoder();
    float value = encoderValue * step / coeficient;

    return value;
  }
};

} // namespace COMPONENT

#endif