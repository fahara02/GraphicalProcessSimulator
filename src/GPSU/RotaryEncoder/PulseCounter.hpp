#ifndef PULSE_COUNTER_HPP
#define PULSE_COUNTER_HPP

#include "GPSU_Defines.hpp"
#include "TimerInfo.hpp"
#include "driver/timer.h"
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
using namespace GPSU_CORE;
struct pcnt_evt_t {
  int unit;
  uint32_t status;
  unsigned long timeStamp;
};

struct pcnt_range_t {
  int minLimit = INT16_MIN;
  int maxLimit = INT16_MAX;
  int thresh0 = -50;
  int thresh1 = 50;
};

class PulseCounter {
public:
  using EncoderISRCallback = std::function<void(void *)>;

  static constexpr uint16_t MAX_ENCODERS = PCNT_UNIT_MAX;
  static constexpr uint16_t PCNT_EVT_QUEUE_SIZE = 10;

  PulseCounter(bool interrupt_enable = false,
               EncoderISRCallback isr_callback = nullptr,
               void *isr_callback_data = nullptr,
               PullType pull_type = PullType::EXTERNAL_PULLUP);

  ~PulseCounter();
  static std::array<std::unique_ptr<PulseCounter>, MAX_ENCODERS> encoders_;
  void attach(int a_pin, int b_pin, pcnt_range_t range = {},
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
  bool isInterreptForAllPulseEnabled() { return all_pulse_interrupt_enabled_; }

  int64_t incrementCount(int64_t delta);
  EncoderISRCallback isr_callback_;
  void *isr_callback_data_;
  QueueHandle_t pcnt_evt_queue_;
  TaskHandle_t pcnt_evt_task;
  static std::atomic<unsigned long> time_counter;

private:
  gpio_num_t sig_io_;
  gpio_num_t cntrl_io_;
  pcnt_unit_t unit_;
  pcnt_config_t pcnt_config_;
  pcnt_range_t pcnt_range_;
  PullType pull_type_;
  EncoderType encoder_type_;
  int counts_mode_;
  std::atomic<int64_t> count_;
  bool all_pulse_interrupt_enabled_;
  bool attached_;
  bool direction_;
  bool working_;

  static uint32_t isr_service_cpu_core_;
  static bool attached_interrupt_;

  void setupTimer();
  void init();
  void configureGPIOs();
  void configurePCNT(EncoderType et, pcnt_range_t range);
  int64_t getRawCount() const;
  bool installInterruptService();

  static void pcnt_intr_handler(void *arg);
  static void IRAM_ATTR onTimer(void *arg);
  static void pcntmonitorTask(void *param);
};

} // namespace COMPONENT

#endif // PULSE_COUNTER_HPP