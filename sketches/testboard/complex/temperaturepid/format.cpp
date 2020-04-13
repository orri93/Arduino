#include "format.h"
#include "text.h"

#define GOS_TEXT_BUFFER(n,t) Holder n(t,sizeof(t))

namespace gatl = ::gos::atl;
namespace gatlf = ::gos::atl::format;

namespace gt = ::gos::temperature;

namespace gos {
namespace temperature {

namespace format {
namespace display {
namespace buffer {
Holder first;
Holder second;
namespace text {
GOS_TEXT_BUFFER(interval, GOS_TCT_INTERVAL);
GOS_TEXT_BUFFER(manual, GOS_TCT_MANUAL);
GOS_TEXT_BUFFER(setpoint, GOS_TCT_SETPOINT);
GOS_TEXT_BUFFER(temperature, GOS_TCT_TEMPERATURE);
GOS_TEXT_BUFFER(kp, GOS_TCT_KP);
GOS_TEXT_BUFFER(ki, GOS_TCT_KI);
GOS_TEXT_BUFFER(kd, GOS_TCT_KD);
GOS_TEXT_BUFFER(minsens, GOS_TCT_MIN_SENS);
GOS_TEXT_BUFFER(maxsens, GOS_TCT_MAX_SENS);
GOS_TEXT_BUFFER(tunetimeunit, GOS_TCT_TUNE_TIME);
GOS_TEXT_BUFFER(force, GOS_TCT_FORCE);
namespace unit {
namespace degree {
GOS_TEXT_BUFFER(centigrade, GOS_TCT_CENTIGRADE);
} // namespace degree
} // namespace unit
namespace codes {
GOS_TEXT_BUFFER(belowrange, GOS_TCT_BELOW_RANGE);
GOS_TEXT_BUFFER(aboverange, GOS_TCT_ABOVE_RANGE);
} // namespace codes
} // namespace text
} // namespace buffer
} // namespace display

namespace real {
::gos::atl::format::option::Number temperature;
::gos::atl::format::option::Number setpoint;
::gos::atl::format::option::Number tune;
}

void initialize() {
  gt::format::real::temperature.Precision = 2;
  gt::format::real::setpoint.Precision = 2;
  gt::format::real::tune.Precision = 3;

  gt::format::display::buffer::first.Buffer[0] = '\0';
  gt::format::display::buffer::second.Buffer[0] = '\0';
}


} // namespace format

} // namespace temperature
} // namespace gos
