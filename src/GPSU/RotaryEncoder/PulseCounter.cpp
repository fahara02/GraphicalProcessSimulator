#include "PulseCounter.hpp"

using namespace COMPONENT;

uint32_t PulseCounter::isrServiceCpuCore = 0;
bool PulseCounter::attachedInterrupt = false;
PulseCounter::PulseCounter(uint16_t max_count, uint16_t min_count,
                           bool intr_enable, enc_isr_cb_t enc_isr_cb,
                           void *enc_isr_cb_data, PullType pull)
    : maxCount_(max_count), minCount_(min_count), enableInterrupt_(intr_enable),
      encoder_isr_callback_(enc_isr_cb),
      encoder_isr_callback_data_(enc_isr_cb_data), encoderPull_(pull) {}

PulseCounter::~PulseCounter() { detach(); }

void PulseCounter::intGPIOs() {

  gpio_pad_select_gpio(aPin_);
  gpio_pad_select_gpio(bPin_);
  gpio_set_direction(aPin_, GPIO_MODE_INPUT);
  gpio_set_direction(bPin_, GPIO_MODE_INPUT);

  if (encoderPull_ == PullType::INTERNAL_PULLUP) {
    gpio_pullup_en(aPin_);
    gpio_pullup_en(bPin_);
  } else if (encoderPull_ == PullType::INTERNAL_PULLDOWN) {
    gpio_pulldown_en(aPin_);
    gpio_pulldown_en(bPin_);
  } else if (encoderPull_ == PullType::NONE) {
    gpio_pullup_dis(aPin_);
    gpio_pulldown_dis(aPin_);
    gpio_pullup_dis(bPin_);
    gpio_pulldown_dis(bPin_);
  }
}

void PulseCounter::configurePCNT(EncoderType et) {
  // Channel 0 configuration
  encoderConfig_.pulse_gpio_num = aPin_;
  encoderConfig_.ctrl_gpio_num = bPin_;
  encoderConfig_.channel = PCNT_CHANNEL_0;
  encoderConfig_.unit = unit_;
  encoderConfig_.pos_mode =
      (et == EncoderType::SINGLE) ? PCNT_COUNT_DEC : PCNT_COUNT_DIS;
  encoderConfig_.neg_mode = PCNT_COUNT_INC;
  encoderConfig_.lctrl_mode = PCNT_MODE_KEEP;
  encoderConfig_.hctrl_mode = PCNT_MODE_REVERSE;
  encoderConfig_.counter_h_lim = maxCount_;
  encoderConfig_.counter_l_lim = minCount_;

  pcnt_unit_config(&encoderConfig_);

  // Channel 1 configuration for FULL encoder type
  if (et == EncoderType::FULL) {
    encoderConfig_.pulse_gpio_num = bPin_;
    encoderConfig_.ctrl_gpio_num = aPin_;
    encoderConfig_.channel = PCNT_CHANNEL_1;
    encoderConfig_.pos_mode = PCNT_COUNT_DEC;
    encoderConfig_.neg_mode = PCNT_COUNT_INC;
    encoderConfig_.lctrl_mode = PCNT_MODE_REVERSE;
    encoderConfig_.hctrl_mode = PCNT_MODE_KEEP;

    pcnt_unit_config(&encoderConfig_);
  }
}