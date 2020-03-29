#include "pid.h"

namespace gt = ::gos::temperature;

namespace gos {
namespace temperature {
namespace pid {

Parameter parameter;
Variable variable;

namespace tune {
Tune k;
TimeTune t;
} // namespace tune

} // namespace pid
} // namespace temperature
} // namespace gos
