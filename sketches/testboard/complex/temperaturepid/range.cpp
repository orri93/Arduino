#include "range.h"
#include "macro.h"

namespace gatl = ::gos::atl;
namespace gatlt = ::gos::atl::type;

namespace gos {
namespace temperature {
namespace range {

Real setpoint = gatlt::make_range<type::Real>(
  SETPOINT_MININUM,
  SETPOINT_MAXIMUM);
Unsigned output = gatlt::make_range<type::Unsigned>(
  PID_MINIMUM_OUTPUT,
  PID_MAXIMUM_OUTPUT);

} // namespace range
} // namespace temperature
} // namespace gos
