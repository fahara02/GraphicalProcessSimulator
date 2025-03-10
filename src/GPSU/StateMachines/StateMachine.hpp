
#pragma once
#include <array>
#include <atomic>
#include <functional>

namespace SM {

template <typename Traits, size_t NumTransitions> class StateMachine {
public:
  using Context = typename Traits::Context;
  using Config = typename Traits::Context::Config;
  using Data = typename Traits::Context::Data;
  using Inputs = typename Traits::Context::Inputs;
  using State = typename Traits::State;
  using Command = typename Traits::Command;
  using Transition = typename Traits::Transition;

  StateMachine(const Context &ctx, State initial)
      : ctx_(ctx), current_(initial), previous_(initial) {
    LoadTransitions();
  }

  void updateData(const Inputs &newInput) {
    input_ = newInput;
    updateInternalState(input_);
  }

  Command update() {
    Command cmd;

    for (const auto &t : transitions_) {
      if (t.from == current() && t.condition && t.condition(ctx_)) {
        if (t.action)
          cmd = t.action(ctx_);
        previous_.store(current_);
        current_.store(t.to);
        notify(previous_, current_);
        break;
      }
    }
    return cmd;
  }

  State current() const { return current_.load(); }
  State previous() const { return previous_.load(); }

  void register_callback(void (*cb)(State, State, const Context &)) {
    callbacks_[callback_count_++] = cb;
  }
  virtual void updateInternalState(const Inputs &newInput) = 0;

protected:
  Context ctx_;
  Inputs input_;
  Data data_;

private:
  std::atomic<State> current_;
  std::atomic<State> previous_;
  std::array<Transition, NumTransitions> transitions_;
  std::array<void (*)(State, State, const Context &), 4> callbacks_{};
  size_t transitionCount_ = 0;
  size_t callback_count_ = 0;

  void notify(State from, State to) {
    for (auto &cb : callbacks_) {
      if (cb)
        cb(from, to, ctx_);
    }
  }

  constexpr void LoadTransitions() {
    for (size_t i = 0; i < NumTransitions; ++i) {
      transitions_[i] = Traits::transitions[i];
    }
    transitionCount_ = NumTransitions;
  }
};
} // namespace SM

// #ifndef STATE_MACHINE_HPP
// #define STATE_MACHINE_HPP

// #include "StateDefines.hpp"
// #include <array>
// #include <atomic>
// #include <functional>

// namespace SM {

// template <typename StateEnum, typename Config, typename StateData,
//           typename Inputs, typename Command>
// struct Transition {
//   uint8_t from;
//   uint8_t to;
//   bool (*condition)(const StateEnum, const StateData &, const Inputs &,
//                     const Config &);
//   Command (*action)(const Inputs &, const Config &);
// };

// template <typename Derived, typename StateEnum, typename StateConfig,
//           typename StateData, typename Inputs, typename Command,
//           typename CommandType,
//           const Transition<StateEnum, StateConfig, StateData, Inputs,
//           Command>
//               *TransitionTable,
//           size_t NumTransitions>
// class StateMachine {
// public:
//   using GuardFunc = bool (*)(const StateEnum, const StateData &, const Inputs
//   &,
//                              const StateConfig &);
//   using ActionFunc = Command (*)(const Inputs &, const StateConfig &);
//   using TransitionCallback = std::function<void(
//       StateEnum, StateEnum, const Inputs &, const StateConfig &)>;
//   constexpr StateMachine(StateEnum initState, const StateConfig config,
//                          const Inputs &initData)
//       : currentState_(initState), config_(config), inputData_(initData) {
//     LoadTransitions();
//   }

//   constexpr StateEnum getState() const { return currentState_; }
//   constexpr const StateData &getData() const { return data_; }
//   void registerTransitionCallback(TransitionCallback cb) {
//     callbacks_.push_back(cb);
//   }
//   void updateData(const Inputs &newData) {
//     inputData_ = newData;
//     updateInternalState(newData);
//   }
//   Command update() {

//     Command cmd{CommandType::NONE};
//     for (const auto &t : transitions_) {
//       if (t.from == static_cast<uint8_t>(currentState_.load())) {
//         if (t.condition(oldState_.load(), data_, inputData_, config_)) {
//           if (t.action)
//             cmd = t.action(inputData_, config_);
//           oldState_.store(currentState_.load());
//           currentState_.store(static_cast<StateEnum>(t.to));
//           inputData_ = Inputs{};
//           notifyTransition(oldState_, static_cast<StateEnum>(t.to));
//           break;
//         }
//       }
//     }
//     return cmd;
//   }
//   void setConfig(const StateConfig &config) { config_ = config; }
//   virtual void updateInternalState(const Inputs &newData) = 0;

// protected:
//   std::atomic<StateEnum> oldState_;
//   std::atomic<StateEnum> currentState_;
//   StateConfig config_;
//   Inputs inputData_;
//   Command command_;
//   StateData data_;

// private:
//   std::array<Transition<StateEnum, StateConfig, StateData, Inputs, Command>,
//              NumTransitions>
//       transitions_;
//   std::vector<TransitionCallback> callbacks_;
//   size_t transitionCount = 0;

//   constexpr void LoadTransitions() {
//     for (size_t i = 0; i < NumTransitions; ++i) {
//       transitions_[i] = TransitionTable[i];
//     }
//     transitionCount = NumTransitions;
//   }
//   void notifyTransition(StateEnum from, StateEnum to) {
//     for (auto &cb : callbacks_) {
//       cb(from, to, inputData_, config_);
//     }
//   }
// };

// } // namespace SM

// #endif

// constexpr void addTransition(StateEnum from, StateEnum to,
//     ConditionFunc condition,
//     ActionFunc action = nullptr) {
// assert(transitionCount < NumTransitions);
// transitions[transitionCount++] = {from, to, condition, action};
// }
