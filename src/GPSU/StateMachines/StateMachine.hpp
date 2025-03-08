#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include "StateDefines.hpp"
#include <array>
#include <atomic>
#include <functional>

namespace SM {

template <typename StateData> struct Transition {
  uint8_t from;
  uint8_t to;
  bool (*condition)(const StateData &);
  void (*action)(StateData &);
};

template <typename Derived, typename StateEnum, typename StateConfig,
          typename StateData, const Transition<StateData> *TransitionTable,
          size_t NumTransitions>
class StateMachine {
public:
  using GuardFunc = bool (*)(const StateData &);
  using ActionFunc = void (*)(StateData &);

  constexpr StateMachine(StateEnum initState, const StateData &initData)
      : currentState(initState), data(initData) {
    LoadTransitions();
  }

  constexpr StateEnum getState() const { return currentState; }
  constexpr const StateData &getData() const { return data; }

  void update() {
    for (size_t i = 0; i < transitionCount; ++i) {
      const auto &t = transitions[i];
      if (t.from == static_cast<uint8_t>(currentState.load()) &&
          t.condition(data)) {
        if (t.action) {
          t.action(data);
        }
        currentState.store(static_cast<StateEnum>(t.to));
        break;
      }
    }
  }

protected:
  std::atomic<StateEnum> currentState;
  StateConfig config;
  StateData data;

private:
  std::array<Transition<StateData>, NumTransitions> transitions;
  size_t transitionCount = 0;

  constexpr void LoadTransitions() {
    for (size_t i = 0; i < NumTransitions; ++i) {
      transitions[i] = TransitionTable[i];
    }
    transitionCount = NumTransitions;
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
