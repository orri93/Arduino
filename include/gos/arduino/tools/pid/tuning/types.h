#ifndef GOS_ARDUINO_TOOL_PID_TUNING_TYPES_H_
#define GOS_ARDUINO_TOOL_PID_TUNING_TYPES_H_

#include <gos/arduino/tools/types.h>
#include <gos/arduino/tools/pid/types.h>

namespace gos {
namespace arduino {
namespace tools {
namespace pid {
namespace tuning {
namespace types {

typedef ::gos::arduino::tools::types::range<
  ::gos::arduino::tools::pid::types::Real> Range;

} // namespace types
} // namespace tuning
} // namespace pid
} // namespace tools
} // namespace arduino 
} // namespace gos

#endif
