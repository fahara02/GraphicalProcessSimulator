#ifndef PULSE_COUNTER_HPP
#define PULSE_COUNTER_HPP

#include <array>
#include <atomic>
#include <climits>
#include <driver/pcnt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portable.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <functional>
#include <memory>
#include <rom/gpio.h>
#include <soc/pcnt_struct.h>

#define ISR_CORE_USE_DEFAULT (0xffffffff)
static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
#define _ENTER_CRITICAL() portENTER_CRITICAL_SAFE(&spinlock)
#define _EXIT_CRITICAL() portEXIT_CRITICAL_SAFE(&spinlock)

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

  int64_t pauseCount();
  int64_t resumeCount();

  void setFilter(uint16_t value);
  bool isAttached() const;

  pcnt_config_t getPCNTconfig() { return pcnt_config_; }
  bool isInterreptForAllPulseEnabled() { return interrupt_enabled_all_pulse_; }

  int64_t incrementCount(int64_t delta);

  EncoderISRCallback isr_callback_;
  void *isr_callback_data_;

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
  std::atomic<int64_t> count_;
  bool interrupt_enabled_all_pulse_;
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