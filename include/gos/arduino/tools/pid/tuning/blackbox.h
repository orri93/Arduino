#ifndef GOS_ARDUINO_TOOL_PID_TUNING_BLACK_BOX_H_
#define GOS_ARDUINO_TOOL_PID_TUNING_BLACK_BOX_H_

#include <gos/arduino/tools/pid/tuning/types.h>
#include <gos/arduino/tools/pid/types.h>

namespace gos {
namespace arduino {
namespace tools {
namespace pid {
namespace tuning {
namespace black {
namespace box {

struct Parameters {
  ::gos::arduino::tools::pid::types::Real Sd;
  ::gos::arduino::tools::pid::tuning::types::Range Kp;
  ::gos::arduino::tools::pid::tuning::types::Range Ki;
};

struct Variables {
  ::gos::arduino::tools::pid::types::Real Kp;
  ::gos::arduino::tools::pid::types::Real Ki;
};

void initialize(
  const Parameters& parameters,
  Variables& variables,
  const ::gos::arduino::tools::pid::types::Real& kp,
  const ::gos::arduino::tools::pid::types::Real& ki);

void initialize(const Parameters& parameters, Variables& variables);

void seed(const ::gos::arduino::tools::pid::types::Real& seed);

namespace random {
::gos::arduino::tools::pid::types::Real next();
}

namespace compute {
void newtunings(const Parameters& parameters, Variables& variables);
}

} // namespace box
} // namespace black
} // namespace tuning
} // namespace pid
} // namespace tools
} // namespace arduino 
} // namespace gos

#endif
