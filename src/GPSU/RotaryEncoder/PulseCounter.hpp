#ifndef PULSE_COUNTER_HPP
#define PULSE_COUNTER_HPP

#include <array>
#include <atomic>
#include <climits>
#include <driver/pcnt.h>
#include <functional>
#include <memory>
#include <soc/pcnt_struct.h>

#define ISR_CORE_USE_DEFAULT (0xffffffff)
namespace COMPONENT {

enum class PullType {
  INTERNAL_PULLUP,
  INTERNAL_PULLDOWN,
  EXTERNAL_PULLUP,
  EXTERNAL_PULLDOWN,
  NONE
};

enum class EncoderType { SINGLE, HALF, FULL };

class PulseCounter {
public:
  using EncoderISRCallback = std::function<void(void *)>;

  static constexpr uint16_t MAX_ENCODERS = PCNT_UNIT_MAX;

  PulseCounter(uint16_t max_count = INT16_MAX, uint16_t min_count = INT16_MIN,
               bool interrupt_enable = false,
               EncoderISRCallback isr_callback = nullptr,
               void *isr_callback_data = nullptr,
               PullType pull_type = PullType::EXTERNAL_PULLUP);

  ~PulseCounter();

  void attach(int a_pin, int b_pin,
              EncoderType encoder_type = EncoderType::FULL);
  void detach();

  int64_t getCount() const;
  void setCount(int64_t value);
  int64_t clearCount();

  void pauseCount();
  void resumeCount();

  void setFilter(uint16_t value);
  bool isAttached() const;

private:
  gpio_num_t a_pin_;
  gpio_num_t b_pin_;
  uint16_t max_count_;
  uint16_t min_count_;
  pcnt_unit_t unit_;
  pcnt_config_t pcnt_config_;
  PullType pull_type_;
  EncoderType encoder_type_;
  int counts_mode_;
  volatile int64_t count_;
  EncoderISRCallback isr_callback_;
  bool interrupt_enabled_;
  void *isr_callback_data_;
  bool attached_;
  bool direction_;
  bool working_;

  static uint32_t isr_service_cpu_core_;
  static bool attached_interrupt_;
  static std::array<std::unique_ptr<PulseCounter>, MAX_ENCODERS> encoders_;

  void initialize();
  void configureGPIOs();
  void configurePCNT(EncoderType et);
  int64_t getRawCount() const;
  bool installInterruptService();
};

} // namespace COMPONENT

#endif // PULSE_COUNTER_HPP