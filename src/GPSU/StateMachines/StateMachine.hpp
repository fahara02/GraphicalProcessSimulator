#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include "StateDefines.hpp"
#include <array>
#include <atomic>
#include <functional>

namespace SM {

template <typename StateData, typename StateConfig> struct Transition {
  uint8_t from;
  uint8_t to;
  bool (*condition)(const StateData &, const StateConfig &);
  void (*action)(StateData &, const StateConfig &);
};

template <typename Derived, typename StateEnum, typename StateConfig,
          typename StateData,
          const Transition<StateData, StateConfig> *TransitionTable,
          size_t NumTransitions>
class StateMachine {
public:
  using GuardFunc = bool (*)(const StateData &, const StateConfig &);
  using ActionFunc = void (*)(StateData &, const StateConfig &);
  using TransitionCallback = std::function<void(
      StateEnum, StateEnum, const StateData &, const StateConfig &)>;
  constexpr StateMachine(StateEnum initState, const StateData &initData)
      : currentState_(initState), data_(initData) {
    LoadTransitions();
  }

  constexpr StateEnum getState() const { return currentState_; }
  constexpr const StateData &getData() const { return data_; }
  void registerTransitionCallback(TransitionCallback cb) {
    callbacks_.push_back(cb);
  }
  void updateData(StateData &newData) { data_ = newData; }
  void update() {
    StateEnum oldState = currentState_;

    for (const auto &t : transitions_) {
      if (t.from == static_cast<uint8_t>(oldState)) {
        if (t.condition(data_, config_)) {
          if (t.action)
            t.action(data_, config_);
          currentState_.store(static_cast<StateEnum>(t.to));
          notifyTransition(oldState, static_cast<StateEnum>(t.to));
          break;
        }
      }
    }
  }
  void setConfig(const StateConfig &config) { config_ = config; }

protected:
  std::atomic<StateEnum> currentState_;
  StateConfig config_;
  StateData data_;

private:
  std::array<Transition<StateData, StateConfig>, NumTransitions> transitions_;
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
      cb(from, to, data_, config_);
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
