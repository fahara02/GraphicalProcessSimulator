
#pragma once
#include <array>
#include <atomic>
#include <cstddef>
#include <functional>
#include <utility>

namespace SM {
template <typename Transition, std::size_t MaxTransitions>
struct TransitionGroup {
  const Transition *transitions[MaxTransitions] = {};
  std::size_t count = 0;
};

template <typename Transition, std::size_t N,
          const std::array<Transition, N> &Transitions>
constexpr std::size_t compute_state_count() {
  std::size_t max_state = 0;
  for (std::size_t i = 0; i < N; ++i) {
    std::size_t s = static_cast<std::size_t>(Transitions[i].from);
    if (s > max_state)
      max_state = s;
  }
  return max_state + 1;
}

template <typename Transition, std::size_t N,
          const std::array<Transition, N> &Transitions>
constexpr auto group_transitions_by_state() {
  constexpr std::size_t state_count =
      compute_state_count<Transition, N, Transitions>();
  std::array<TransitionGroup<Transition, N>, state_count> groups{};
  for (std::size_t i = 0; i < N; ++i) {
    const auto &t = Transitions[i];
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
  using State = typename Traits::State;
  using Command = typename Traits::Command;
  using Transition = typename Traits::Transition;

  StateMachine(const Context &ctx, State initial)
      : ctx_(ctx), current_(initial), previous_(initial) {}

  Command update() {
    Command cmd{};
    if (!dataUpdated_)
      return cmd;
    dataUpdated_ = false;
    constexpr auto lookup = Traits::transitions_by_state;
    const auto current_state = static_cast<std::size_t>(current());
    if (current_state < lookup.size()) {
      const auto &group = lookup[current_state];
      for (std::size_t i = 0; i < group.count; ++i) {
        const auto *t = group.transitions[i];
        if (t->condition && t->condition(ctx_)) {
          cmd = executeTransition(*t);
          previous_.store(current());
          current_.store(t->to);
          ctx_.previous_state = previous_.load();
          notify(previous_, current_);
          return cmd;
        }
      }
    }
    return cmd;
  }

  State current() const { return current_.load(); }
  State previous() const { return previous_.load(); }

  void register_callback(void (*cb)(State, State, Context &)) {
    if (callback_count_ < callbacks_.size()) {
      callbacks_[callback_count_++] = cb;
    }
  }
  void updateData(const Inputs &newInput) {
    static_cast<Derived *>(this)->updateInternalState(newInput);
    dataUpdated_ = true;
  }

protected:
  Context ctx_;

private:
  std::atomic<State> current_;
  std::atomic<State> previous_;

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
  // Helper to count transitions per 'from' state at compile time
  template <typename Transition, size_t N, typename State>
  constexpr auto
  count_transitions_by_from(const std::array<Transition, N> &transitions,
                            State from_state) {
    size_t count = 0;
    for (size_t i = 0; i < N; ++i) {
      if (transitions[i].from == from_state) {
        ++count;
      }
    }
    return count;
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