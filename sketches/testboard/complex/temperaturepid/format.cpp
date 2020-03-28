#include "format.h"
#include "text.h"

#define GOS_TEXT_BUFFER(n,t) Holder n(t,sizeof(t))

namespace gatl = ::gos::atl;
namespace gatlf = ::gos::atl::format;

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
namespace unit {
namespace degree {
GOS_TEXT_BUFFER(centigrade, GOS_TCT_CENTIGRADE);
} // namespace degree
} // namespace unit
} // namespace text
} // namespace buffer
} // namespace display

namespace real {
gatlf::option::Number option;
}

} // namespace format

} // namespace temperature
} // namespace gos
