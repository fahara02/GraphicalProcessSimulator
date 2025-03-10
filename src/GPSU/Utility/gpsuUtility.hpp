#ifndef GPSU_UTILITY_HPP
#define GPSU_UTILITY_HPP

#include "Process/ProcessDefines.hpp"

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