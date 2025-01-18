#ifndef PULSE_COUNTER_HPP
#define PULSE_COUNTER_HPP
#include <climits>
#include <driver/pcnt.h>
#include <memory>
#include <soc/pcnt_struct.h>

namespace COMPONENT {

enum class PullType {
  INTERNAL_PULLUP = 0,
  INTERNAL_PULLDOWN = 1,
  EXTERNAL_PULLUP = 2,
  EXTERNAL_PULLDOWN = 3,
  NONE = 4
};

enum class EncoderType { SINGLE = 0, HALF = 1, FULL = 2 };
class PulseCounter {
  typedef void (*enc_isr_cb_t)(void *);
  static constexpr uint16_t MAX_ENCODERS = PCNT_UNIT_MAX;

private:
  gpio_num_t aPin_;
  gpio_num_t bPin_;
  uint16_t maxCount_ = INT16_MAX;
  uint16_t minCount_ = INT16_MIN;
  pcnt_unit_t unit_;
  pcnt_config_t encoderConfig_;
  PullType encoderPull_;
  EncoderType encoderType_;
  int countsMode_ = 2;
  volatile int64_t count_ = 0;
  enc_isr_cb_t encoder_isr_callback_;
  bool enable_interrupt_;
  void *encoder_isr_callback_data_;
  static uint32_t isrServiceCpuCore;

  std::array<std::unique_ptr<PulseCounter>, MAX_ENCODERS> encoders_ = {};

public:
  PulseCounter(uint16_t max_count = INT16_MAX, uint16_t min_count = INT16_MIN,
               bool intr_enable = false, enc_isr_cb_t enc_isr_cb = nullptr,
               void *enc_isr_cb_data = nullptr,
               PullType pull = PullType::EXTERNAL_PULLUP);
  ~PulseCounter();

  void attach(int aPin, int bPin, EncoderType et = EncoderType::FULL);

  int64_t getCount();
  int64_t clearCount();
  int64_t pauseCount();
  int64_t resumeCount();
  void detach();
  void setCount(int64_t value);
  void setFilter(uint16_t value);
  bool isAttached() { return attached; }

private:
  static bool attachedInterrupt;
  int64_t getCountRaw();
  void init();
  void intGPIOs();
  void configurePCNT(EncoderType et);
  bool attached;
  bool direction;
  bool working;
};

} // namespace COMPONENT
#endif