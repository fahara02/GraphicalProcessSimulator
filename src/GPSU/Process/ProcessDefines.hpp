#ifndef PROCESS_DEFINES_HPP
#define PROCESS_DEFINES_HPP
namespace GPSU {
enum class ProcessType {
  TRAFFIC_LIGHT = 0,
  WATER_LEVEL = 1,
  STEPPER_MOTOR_CONTROL = 2,
  STATE_MACHINE = 3,
  OBJECT_COUNTER = 4,
  MOTOR_CONTROl = 5,
  ANY = -1,

};
}
#endif