
#pragma once
#include "Logger.hpp"
#include "Utility/gpsuUtility.hpp"
#include "esp_log.h"
#include <array>
#include <atomic>
#include <cstddef>
#include <esp_timer.h>
#include <functional>
#include <utility>

namespace SM {

template <typename Transition, std::size_t MaxTransitions>
struct TransitionGroup {
  const Transition *transitions[MaxTransitions] = {};
  std::size_t count = 0;
};
template <typename Transition, std::size_t N>
constexpr std::size_t
compute_state_count(const std::array<Transition, N> &transitions) {
  std::size_t max_state = 0;
  for (std::size_t i = 0; i < N; ++i) {
    std::size_t s = static_cast<std::size_t>(transitions[i].from);
    if (s > max_state) {
      max_state = s;
    }
  }
  return max_state + 1;
}

template <typename Transition, std::size_t N, std::size_t StateCount>
constexpr std::array<TransitionGroup<Transition, N>, StateCount>
group_transitions_by_state(const std::array<Transition, N> &transitions) {
  std::array<TransitionGroup<Transition, N>, StateCount> groups{};
  for (std::size_t i = 0; i < N; ++i) {
    const auto &t = transitions[i];
    std::size_t state = static_cast<std::size_t>(t.from);
    groups[state].transitions[groups[state].count] = &t;
    ++groups[state].count;
  }
  return groups;
}

template <typename Traits, typename Derived> class StateMachine {
public:
  using Context = typename Traits::Context;
  using Mode = typename Traits::Context::Mode;
  using Inputs = typename Traits::Context::Inputs;
  using Event = typename Traits::Context::Event;
  using State = typename Traits::Context::State;
  using Command = typename Traits::Context::Command;
  using Transition = typename Traits::Transition;

  explicit StateMachine(const Context &ctx, State initial, bool enableTimer,
                        bool enableTimer2 = false)
      : ctx_(ctx), current_(initial), previous_(initial),
        enable_timer_(enableTimer), enable_timer2_(enableTimer2) {}
  ~StateMachine() {
    is_destroying_ = true;
    if (enable_timer_ && timer_) {
      esp_timer_stop(timer_);
      esp_timer_delete(timer_);
      timer_ = nullptr;
    }
    if (enable_timer2_ && timer2_) {
      esp_timer_stop(timer2_);
      esp_timer_delete(timer2_);
      timer2_ = nullptr;
    }
  }
  Command update() {

    Command cmd{};
    if (!dataUpdated_) {
      return cmd;
    }
    dataUpdated_ = false;

    const auto curr = static_cast<std::size_t>(current());
    if (curr < Traits::transitions_by_state.size()) {
      const auto &group = Traits::transitions_by_state[curr];
      for (std::size_t i = 0; i < group.count; ++i) {
        const auto *t = group.transitions[i];
        if (t->condition && t->condition(ctx_)) {
          cmd = executeTransition(*t);
          previous_.store(current());
          current_.store(t->to);
          ctx_.curr = current_.load();
          ctx_.prev = previous_.load();
          ctx_.new_cmd = cmd;
          notify(previous_, current_);
          handleTimer(cmd);
          return cmd;
        }
      }
    }
    return cmd;
  }
  Context context() const { return ctx_; }
  State current() const { return current_.load(); }
  State previous() const { return previous_.load(); }

  void register_callback(void (*cb)(State, State, Context &)) {
    if (callback_count_ < callbacks_.size()) {
      callbacks_[callback_count_++] = cb;
    }
  }
  Command updateData() {
    static_cast<Derived *>(this)->updateInternalState(ctx_.inputs);
    dataUpdated_ = true;
    return update();
  }
  Command updateData(const Inputs &newInput) {

    ctx_.inputs = newInput;
    static_cast<Derived *>(this)->updateInternalState(newInput);
    dataUpdated_ = true;
    return update();
  }
  void handleEvent(const Event ev) {
    static_cast<Derived *>(this)->updateInternalState(ev);
    update();
  }
  void init() {
    if (!initialised) {
      initialised = true;

      createTimers();
    }
  }
  const Context &getContext() const { return ctx_; }
  void setContext(Context ctx) { ctx_ = ctx; }
  void setAutoUpdate() { dataUpdated_ = true; }
  void stopTimers() {
    if (enable_timer_ && timer_) {
      esp_timer_stop(timer_);
    }
    if (enable_timer2_ && timer2_) {
      esp_timer_stop(timer2_);
    }
  }

protected:
  Context ctx_;
  bool timer1_was_deleted = false;
  bool timer2_was_deleted = false;



  void createTimers() {
    dataUpdated_ = true;
    if (enable_timer_) {
      LOG::INFO("SM", "Creating Timer 1");
      esp_timer_create_args_t timer1_args = {.callback = &timer1Callback,
                                             .arg = this,
                                             .dispatch_method = ESP_TIMER_TASK,
                                             .name = "sm_timer1"};
      esp_err_t err = esp_timer_create(&timer1_args, &timer_);
      if (err == ESP_OK) {
        LOG::SUCCESS("SM", "TIMER1 CREATED");
      }
      if (err != ESP_OK || !timer_) {
        LOG::ERROR("SM", "Timer1 creation failed");
      }
    }
    if (enable_timer2_) {
      LOG::INFO("SM", "Creating Timer 2");
      esp_timer_create_args_t timer2_args = {.callback = &timer2Callback,
                                             .arg = this,
                                             .dispatch_method = ESP_TIMER_TASK,
                                             .name = "sm_timer2"};
      esp_err_t err = esp_timer_create(&timer2_args, &timer2_);
      if (err != ESP_OK || !timer2_) {
        LOG::ERROR("SM", "Timer2 creation failed");
      }
    }
  }
  void restartTimer1(uint64_t timeout_us) {
    if (enable_timer_ && timer_) {
      esp_timer_stop(timer_); // Stop if already running
      esp_err_t err = esp_timer_start_once(timer_, timeout_us);
      if (err != ESP_OK) {

        LOG::ERROR("SM", "Timer1 restart failed");
      }
    }
  }
  void disableTimers() {

    if (enable_timer_ && timer_) {
      esp_timer_stop(timer_);
      esp_timer_delete(timer_);
      timer_ = nullptr;
      enable_timer_ = false;
      timer1_was_deleted = true;
    }
    if (enable_timer2_ && timer2_) {
      esp_timer_stop(timer2_);
      esp_timer_delete(timer2_);
      timer2_ = nullptr;
      enable_timer2_ = false;
      timer2_was_deleted = true;
    }
  }
  void enableTimer(int timer_id = 0) {
    if (timer_id == 0) {
      enable_timer_ = true;
    } else if (timer_id == 1) {
      enable_timer2_ = true;
    }
  }

private:
  std::atomic<State> current_;
  std::atomic<State> previous_;
  bool is_destroying_ = false;
  bool enable_timer_ = false;
  bool enable_timer2_ = false;
  bool initialised = false;

  // Timer members
  esp_timer_handle_t timer_;
  esp_timer_handle_t timer2_;

  bool timerExpired_ = false;
  bool timer2Expired_ = false;

  // Timer control 0=timer 1 ,1=timer2
  esp_err_t startTimer(uint64_t timeout_us, int timer_id = 0) {
    if (timer_id == 0) {
      return enable_timer_ ? esp_timer_start_once(timer_, timeout_us)
                           : ESP_FAIL;
    }
    return enable_timer2_ ? esp_timer_start_once(timer2_, timeout_us)
                          : ESP_FAIL;
  }

  void stopTimer(int timer_id = 0) {
    if (timer_id == 0 && enable_timer_) {
      esp_timer_stop(timer_);
    } else if (timer_id == 1 && enable_timer2_) {
      esp_timer_stop(timer2_);
    }
  }

  bool isTimerExpired(int timer_id = 0) const {
    return timer_id == 0 ? timerExpired_ : timer2Expired_;
  }
  void resetTimerExpired(int timer_id = 0) {
    if (timer_id == 0) {
      timerExpired_ = false;
      ctx_.inputs.timer.t1_expired = false;
    } else {
      timer2Expired_ = false;
      ctx_.inputs.timer.t2_expired = false;
    }
  }

  const Transition *transitions_ = Traits::transitions.data();
  std::array<void (*)(State, State, Context &), 4> callbacks_{};
  size_t transitionCount_ = Traits::transition_count;
  size_t callback_count_ = 0;
  volatile bool dataUpdated_ = false;

  void notify(State from, State to) {
    for (size_t i = 0; i < callback_count_; ++i) {
      callbacks_[i](from, to, ctx_);
    }
  }

  Command executeTransition(const Transition &t) {
    Command cmd{};
    if constexpr (Traits::has_exit_actions) {
      if (t.exit_action)
        cmd = mergeCommands(cmd, t.exit_action(ctx_));
    }
    if constexpr (Traits::has_entry_actions) {
      if (t.entry_action)
        cmd = mergeCommands(cmd, t.entry_action(ctx_));
    }
    return cmd;
  }
  Command mergeCommands(Command base, Command next) {
    if (next.check_exit) {
      base.exit_command = next.exit_command;
      base.exit_data = next.exit_data;
      base.check_exit = true;
    }
    if (next.check_entry) {
      base.entry_command = next.entry_command;
      base.entry_data = next.entry_data;
      base.check_entry = true;
    }
    return base;
  }
  static void timer1Callback(void *arg) {
    StateMachine *sm = static_cast<StateMachine *>(arg);
    if (sm && !sm->is_destroying_) {
      sm->timerExpired_ = true;
      sm->ctx_.inputs.timer.t1_expired = true;
      sm->dataUpdated_ = true;
      if (sm->enable_timer_) {
        sm->updateData(sm->ctx_.inputs);
      }
    }
  }

  static void timer2Callback(void *arg) {
    StateMachine *sm = static_cast<StateMachine *>(arg);
    if (sm && !sm->is_destroying_) {
      sm->timer2Expired_ = true;
      sm->ctx_.inputs.timer.t2_expired = true;
      sm->dataUpdated_ = true;
      if (sm->enable_timer2_) {
        sm->updateData(sm->ctx_.inputs);
      }
    }
  }
  void handleTimer(Command cmd) {
    // Handle Timer 1
    if (enable_timer_) {
      resetTimerExpired(0);
      if (esp_timer_is_active(timer_)) {
        stopTimer(0);
      }
      if (cmd.check_entry && cmd.entry_data.timeout1 > 0) {
        esp_err_t err = startTimer(cmd.entry_data.timeout1 * 1000, 0);
        if (err != ESP_OK) {
          LOG::ERROR("SM", "Timer1 start failed");
        }
      } else {
        stopTimer(0);
      }
    }

    // Handle Timer 2
    if (enable_timer2_) {
      resetTimerExpired(1);
      if (cmd.check_entry && cmd.entry_data.timeout2 > 0) {
        esp_err_t err = startTimer(cmd.entry_data.timeout2 * 1000, 1);
        if (err != ESP_OK) {
          LOG::ERROR("SM", "Timer2 start failed");
        }
      } else {
        stopTimer(1);
      }
    }
  }
};
} // namespace SM
