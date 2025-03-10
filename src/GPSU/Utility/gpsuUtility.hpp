#ifndef GPSU_UTILITY_HPP
#define GPSU_UTILITY_HPP

#include "Process/ProcessDefines.hpp"
#include "StateMachines/StateDefines.hpp"

namespace GPSU {

class Util {
public:
  class ToString {
  public:
    static const char *Process(ProcessType type) {

      switch (type) {

      case ProcessType ::TRAFFIC_LIGHT:
        return "Traffic-Light";
        break;
      case ProcessType ::WATER_LEVEL:
        return "Water-Level";
        break;
      case ProcessType ::STEPPER_MOTOR_CONTROL:
        return "Stepper-Motor";
        break;
      case ProcessType ::STATE_MACHINE:
        return "State-Machine";
        break;
      case ProcessType ::OBJECT_COUNTER:
        return "Object-Counter";
        break;
      case ProcessType ::MOTOR_CONTROl:
        return "Motor-Control";
        break;
      default:
        return "Unknown";
        break;
      }
    }
    static const char *TLState(TrafficLight::State state) {
      using State = TrafficLight::State;
      switch (state) {
      case State::INIT:
        return "INIT";
      case State::RED_STATE:
        return "RED";
      case State::GREEN_STATE:
        return "GREEN";
      case State::YELLOW_STATE:
        return "YELLOW";
      default:
        return "UNKNOWN";
      }
    }
    static const char *TLCommands(TrafficLight::CommandType command) {
      using Command = TrafficLight::CommandType;
      switch (command) {
      case Command::NONE:
        return "NONE";
      case Command::TURN_ON_RED:
        return "TURN_ON_RED";
      case Command::TURN_OFF_RED:
        return "TURN_OFF_RED";
      case Command::TURN_ON_YELLOW:
        return "TURN_ON_YELLOW";
      case Command::TURN_OFF_YELLOW:
        return "TURN_OFF_YELLOW";
      case Command::TURN_ON_GREEN:
        return "TURN_ON_GREEN";
      case Command::TURN_OFF_GREEN:
        return "TURN_OFF_GREEN";
      case Command::RESET:
        return "RESET";
      default:
        return "UNKNOWN";
      }
    }

    static const char *TankState(int state) {

      switch (state) {

      case 0:
        return "Draining";
        break;
      case 1:
        return "Filling";
        break;
      case 2:
        return "Full";
        break;
      case 3:
        return "OverFlow";
        break;

      default:
        return "Unknown";
        break;
      }
    }
  };
};

} // namespace GPSU
#endif