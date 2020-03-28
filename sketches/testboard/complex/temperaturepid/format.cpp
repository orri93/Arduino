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
gatlf::option::Number option;
}

void initialize() {
  gt::format::display::buffer::first.Buffer[0] = '\0';
  gt::format::display::buffer::second.Buffer[0] = '\0';
}


} // namespace format

} // namespace temperature
} // namespace gos
