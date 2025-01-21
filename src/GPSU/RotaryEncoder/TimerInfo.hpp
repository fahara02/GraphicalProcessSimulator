#ifndef TIMER_INFO_HPP
#define TIMER_INFO_HPP
#include "driver/timer.h"
#include "stdint.h"
namespace GPSU_CORE {
#define TIMER_DIVIDER (16)
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER)
constexpr double TIMER_INTERVAL_MS = 1000.0;
constexpr uint64_t ALARM_VALUE =
    static_cast<uint64_t>(TIMER_SCALE * (TIMER_INTERVAL_MS / 1000.0));
constexpr timer_group_t TIMER_GROUP = TIMER_GROUP_0;
constexpr timer_idx_t TIMER_INDEX = TIMER_0;

struct timer_info_t {
  int timer_group;
  int timer_idx;
  int alarm_interval;
  bool auto_reload;
};
struct timer_event_t {
  timer_info_t info;
  uint64_t timer_counter_value;
};

} // namespace GPSU_CORE

#endif