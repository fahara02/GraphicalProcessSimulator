
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
    if (!dataUpdated_)
      return Command{};
    dataUpdated_ = false;
    Command cmd;
    bool transitionFound = false;
    for (const auto &t : transitions_) {
      if (t.from == current() && t.condition && t.condition(ctx_)) {
        Command exitCmd, entryCmd;
        // Execute exit action from the current state
        if constexpr (Traits::has_exit_actions) {
          if (t.exit_action) {
            exitCmd = t.exit_action(ctx_);
          }
        }
        // Update to the new state
        previous_.store(current_);
        const State newState = t.to;
        current_.store(newState);
        ctx_.previous_state = previous_.load();
        // Execute entry action for the new state
        if constexpr (Traits::has_entry_actions) {
          if (t.entry_action) {
            entryCmd = t.entry_action(ctx_);
          }
        }

        // Merge exit and entry commands
        if (exitCmd.check_exit) {
          cmd.exit_command = exitCmd.exit_command;
          cmd.exit_data = exitCmd.exit_data;
          cmd.check_exit = true;
        }
        if (entryCmd.check_entry) {
          cmd.entry_command = entryCmd.entry_command;
          cmd.entry_data = entryCmd.entry_data;
          cmd.check_entry = true;
        }
        notify(previous_.load(), newState);
        transitionFound = true;
        break;
      }
    }
    if (!transitionFound) {
      // Serial.println("No valid transition found for state: " +
      //                String((int)current()));
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
};
} // namespace SM
