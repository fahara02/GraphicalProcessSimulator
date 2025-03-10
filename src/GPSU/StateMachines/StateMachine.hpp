
#pragma once
#include <array>
#include <atomic>
#include <functional>

namespace SM {

template <typename Traits, typename Derived> class StateMachine {
public:
  using Context = typename Traits::Context;
  using Inputs = typename Traits::Context::Inputs;
  using State = typename Traits::State;
  using Command = typename Traits::Command;
  using Transition = typename Traits::Transition;

  StateMachine(const Context &ctx, State initial)
      : ctx_(ctx), current_(initial), previous_(initial) {
    LoadTransitions();
  }

  Command update() {
    Command cmd = Command{};
    if (!dataUpdated_)
      return cmd;
    dataUpdated_ = false;

    for (const auto &t : transitions_) {
      if (t.from == current() && t.condition && t.condition(ctx_)) {
        Command cmd = executeTransition(t);
        previous_.store(current());
        current_.store(t.to);
        ctx_.previous_state = previous_.load();
        notify(previous_, current_);
        return cmd;
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
  std::array<Transition, Traits::transition_count> transitions_;
  std::array<void (*)(State, State, Context &), 4> callbacks_{};
  size_t transitionCount_ = 0;
  size_t callback_count_ = 0;
  volatile bool dataUpdated_ = false;
  void notify(State from, State to) {
    for (auto &cb : callbacks_) {
      if (cb)
        cb(from, to, ctx_);
    }
  }

  constexpr void LoadTransitions() {
    for (size_t i = 0; i < Traits::transition_count; ++i) {
      transitions_[i] = Traits::transitions[i];
    }
    transitionCount_ = Traits::transition_count;
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
};
} // namespace SM
