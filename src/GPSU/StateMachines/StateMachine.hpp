
#pragma once
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
  using Inputs = typename Traits::Context::Inputs;
  using Event = typename Traits::Context::Event;
  using State = typename Traits::State;
  using Command = typename Traits::Command;
  using Transition = typename Traits::Transition;

  explicit StateMachine(const Context &ctx, State initial, bool enableTimer)
      : ctx_(ctx), current_(initial), previous_(initial),
        enable_timer_(enableTimer) {}

  Command update() {

    Command cmd{};
    if (!dataUpdated_) {
      return cmd;
    }
    dataUpdated_ = false;
    const auto current_state = static_cast<std::size_t>(current());
    if (current_state < Traits::transitions_by_state.size()) {
      const auto &group = Traits::transitions_by_state[current_state];
      for (std::size_t i = 0; i < group.count; ++i) {
        const auto *t = group.transitions[i];
        if (t->condition && t->condition(ctx_)) {
          cmd = executeTransition(*t);
          previous_.store(current());
          current_.store(t->to);
          ctx_.previous_state = previous_.load();
          notify(previous_, current_);
          handleTimer(cmd);
          return cmd;
        }
      }
    }
    return cmd;
  }
  State current() const { return current_.load(); }
  State previous() const { return previous_.load(); }
  Context context() const { return ctx_; }

  void register_callback(void (*cb)(State, State, Context &)) {
    if (callback_count_ < callbacks_.size()) {
      callbacks_[callback_count_++] = cb;
    }
  }

  void updateData(const Inputs &newInput) {
    ctx_.inputs = newInput;
    static_cast<Derived *>(this)->updateInternalState(newInput);
    dataUpdated_ = true;
  }
  void handleEvent(const Event ev) {
    static_cast<Derived *>(this)->updateInternalState(ev);
    update();
  }
  void init() {

    if (!initialised) {
      initialised = true;
      dataUpdated_ = true;
      if (enable_timer_) {

        Serial.println("Creating Timer");
        esp_timer_create_args_t timer_args = {.callback = &timerCallback,
                                              .arg = this,
                                              .dispatch_method = ESP_TIMER_TASK,
                                              .name = "sm_timer"};
        esp_err_t err = esp_timer_create(&timer_args, &timer_);
        if (err != ESP_OK) {
          Serial.println("Timer creation failed");
        }
        if (!timer_) { // Check if timer is NULL
          Serial.println("Timer handle is NULL, cannot start timer!");
        }
        if (err == ESP_OK && timer_) {
          // ESP_LOGI("STATE_MACHINE", "TIMER_CREATION SUCCESSFULL");
        }
      }
    }
  }

protected:
  Context ctx_;

private:
  std::atomic<State> current_;
  std::atomic<State> previous_;
  bool enable_timer_ = false;
  bool initialised = false;

  // Timer members
  esp_timer_handle_t timer_;

  // Timer control
  esp_err_t startTimer(uint64_t timeout_us) {
    return esp_timer_start_once(timer_, timeout_us);
  }

  void stopTimer() { esp_timer_stop(timer_); }

  bool isTimerExpired() const { return timerExpired_; }
  void resetTimerExpired() {
    timerExpired_ = false;
    ctx_.inputs.timer.internal_timer_expired = false;
  }
  bool timerExpired_ = false;

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

  static void timerCallback(void *arg) {
    StateMachine *sm = static_cast<StateMachine *>(arg);
    sm->timerExpired_ = true;
    sm->ctx_.inputs.timer.internal_timer_expired = true;
    sm->dataUpdated_ = true;
    if (sm->enable_timer_) {
      sm->updateData(sm->ctx_.inputs);
    }
  }
  void handleTimer(Command cmd) {

    if (enable_timer_) {
      if (!timer_) {

        Serial.println("no timer handle yet");
      } else {
        resetTimerExpired();
        if (cmd.check_entry && cmd.entry_data.timeout_ms > 0) {

          esp_err_t err = startTimer(
              static_cast<uint64_t>(cmd.entry_data.timeout_ms) * 1000);

          if (err != ESP_OK) {
            Serial.println("Timer start  failed");
          }
        } else {
          stopTimer();
        }
      }
    }
  }
};
} // namespace SM

// Command update() {
//   Command cmd = Command{};
//   if (!dataUpdated_)
//     return cmd;
//   dataUpdated_ = false;

//   // Modified loop: iterate using transition count and pointer
//   for (size_t i = 0; i < transitionCount_; ++i) { // Iterate by index
//     const auto &t = transitions_[i];              // Access via pointer
//     if (t.from == current() && t.condition && t.condition(ctx_)) {
//       Command cmd = executeTransition(t);
//       previous_.store(current());
//       current_.store(t.to);
//       ctx_.previous_state = previous_.load();
//       notify(previous_, current_);
//       return cmd;
//     }
//   }

//   return cmd;
// }