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
      case ProcessType ::STEPPER_MOTOR:
        return "Stepper-Motor";
        break;
      case ProcessType ::STATE_MACHINE:
        return "State-Machine";
        break;
      case ProcessType ::OBJECT_COUNTER:
        return "Item-Counter";
        break;
      case ProcessType ::MOTOR_CONTROL:
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
      case State::RED:
        return "RED";
      case State::GREEN:
        return "GREEN";
      case State::YELLOW:
        return "YELLOW";
      case State::FAULT:
        return "FAULT";
      default:
        return "UNKNOWN";
      }
    }
    static const char *TLCommands(TrafficLight::CmdType command) {
      using Command = TrafficLight::CmdType;
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
    static const char *WLState(WaterLevel::State state) {
      using State = WaterLevel::State;
      switch (state) {
      case State::EMPTY:
        return "EMPTY";
      case State::START_FILLING:
        return "START_FILLING";
      case State::FILLING:
        return "FILLING";
      case State::DRAINING:
        return "DRAINING";
      case State::PARTIAL_FILLED:
        return "PARTIAL_FILLED";
      case State::FULL:
        return "FULL";
      case State::OVERFLOW:
        return "OVERFLOW";
      case State::FAULT:
        return "FAULT";
      default:
        return "UNKNOWN";
      }
    }
    static const char *OCState(ObjectCounter::State state) {
      using State = ObjectCounter::State;
      switch (state) {
      case State::INIT:
        return "INIT";
      case State::READY:
        return "READY";
      case State::RUNNING:
        return "RUNNING";
      case State::RESET:
        return "RESET";
      case State::FAULT:
        return "FAULT";
      case State::E_STOP:
        return "ESTOP";

      default:
        return "UNKNOWN";
      }
    }
    static const char *OCCommands(ObjectCounter::CmdType cmd) {
      using Command = ObjectCounter::CmdType;
      switch (cmd) {
      case Command::NONE:
        return "NONE";
      case Command::RESET:
        return "RESET";
      case Command::START:
        return "START";
      case Command::STOP:
        return "STOP";
      case Command::NEW_OBJ:
        return "NEW_OBJ";
      case Command::SENSE:
        return "SENSE";
      case Command::PICK:
        return "ACTIVATE_PICKER";
      case Command::ALARM:
        return "ALARM";
      case Command::CLEAR_ALARM:
        return "CLEAR_ALARM";
      case Command::MANUAL_MODE:
        return "MANUAL_MODE";
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