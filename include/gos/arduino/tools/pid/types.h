#ifndef GOS_ARDUINO_TOOL_PID_TYPES_H_
#define GOS_ARDUINO_TOOL_PID_TYPES_H_

#include <cstdint>

namespace gos {
namespace arduino {
namespace tools {
namespace pid {
namespace types {

typedef uint16_t Unsigned;
typedef float Real;

namespace registry {

struct Input {
  Unsigned Output;
  Real Temperature;
  Real Error;
  Real Integral;
  Real Derivative;
  Unsigned Status;
};

struct Holding {
  Unsigned Interval;
  Unsigned Manual;
  Real Setpoint;
  Real Kp;
  Real Ki;
  Real Kd;
  Real SensorMinimum;
  Real SensorMaximum;
  Unsigned TimeTune;
  Unsigned Force;
};

} // namespace registry

} // namespace types
} // namespace pid
} // namespace tools
} // namespace arduino 
} // namespace gos

#endif
