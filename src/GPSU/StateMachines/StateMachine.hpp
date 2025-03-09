#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include "StateDefines.hpp"
#include <array>
#include <atomic>
#include <functional>

namespace SM {

template <typename StateEnum, typename Config, typename Inputs,
          typename Command>
struct Transition {
  uint8_t from;
  uint8_t to;
  bool (*condition)(const StateEnum, const Inputs &, const Config &);
  Command (*action)(const Inputs &, const Config &);
};

template <
    typename Derived, typename StateEnum, typename StateConfig, typename Inputs,
    typename Command, typename CommandType,
    const Transition<StateEnum, StateConfig, Inputs, Command> *TransitionTable,
    size_t NumTransitions>
class StateMachine {
public:
  using GuardFunc = bool (*)(const StateEnum, const Inputs &,
                             const StateConfig &);
  using ActionFunc = Command (*)(const Inputs &, const StateConfig &);
  using TransitionCallback = std::function<void(
      StateEnum, StateEnum, const Inputs &, const StateConfig &)>;
  constexpr StateMachine(StateEnum initState, const StateConfig config,
                         const Inputs &initData)
      : currentState_(initState), config_(config), inputData_(initData) {
    LoadTransitions();
  }

  constexpr StateEnum getState() const { return currentState_; }
  constexpr const Inputs &getInputData() const { return inputData_; }
  void registerTransitionCallback(TransitionCallback cb) {
    callbacks_.push_back(cb);
  }
  void updateData(const Inputs &newData) { inputData_ = newData; }
  Command update() {

    Command cmd{CommandType::NONE};
    for (const auto &t : transitions_) {
      if (t.from == static_cast<uint8_t>(currentState_.load())) {
        if (t.condition(oldState_.load(), inputData_, config_)) {
          if (t.action)
            cmd = t.action(inputData_, config_);
          oldState_.store(currentState_.load());
          currentState_.store(static_cast<StateEnum>(t.to));
          inputData_ = Inputs{};
          notifyTransition(oldState_, static_cast<StateEnum>(t.to));
          break;
        }
      }
    }
    return cmd;
  }
  void setConfig(const StateConfig &config) { config_ = config; }

protected:
  std::atomic<StateEnum> oldState_;
  std::atomic<StateEnum> currentState_;
  StateConfig config_;
  Inputs inputData_;
  Command command_;

private:
  std::array<Transition<StateEnum, StateConfig, Inputs, Command>,
             NumTransitions>
      transitions_;
  std::vector<TransitionCallback> callbacks_;
  size_t transitionCount = 0;

  constexpr void LoadTransitions() {
    for (size_t i = 0; i < NumTransitions; ++i) {
      transitions_[i] = TransitionTable[i];
    }
    transitionCount = NumTransitions;
  }
  void notifyTransition(StateEnum from, StateEnum to) {
    for (auto &cb : callbacks_) {
      cb(from, to, inputData_, config_);
    }
  }
};

} // namespace SM

#endif

// constexpr void addTransition(StateEnum from, StateEnum to,
//     ConditionFunc condition,
//     ActionFunc action = nullptr) {
// assert(transitionCount < NumTransitions);
// transitions[transitionCount++] = {from, to, condition, action};
// }
