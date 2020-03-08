#include "pid.h"

namespace gt = ::gos::temperature;

namespace gos {
namespace temperature {
namespace pid {

Parameter parameter;
Variable variable;

namespace tune {
#ifdef PID_STORE_TIME_TUNE
Tune t;
namespace calculated {
Tune k;
} // namespace calculated
#else
gt::pid::tune::Tune k;
namespace calculated {
gt::pid::tune::Tune t;
} // namespace calculated
#endif
} // namespace tune

} // namespace pid
} // namespace temperature
} // namespace gos
