#include "pid.h"
#include "variable.h"

namespace gt = ::gos::temperature;
namespace gtv = ::gos::temperature::variables;

namespace gatl = ::gos::atl;
namespace gatlp = ::gos::atl::pid;

namespace gos {
namespace temperature {
namespace pid {

Parameter parameter;
Variable variable;

void create() {
  gt::pid::parameter.Range =
    gatl::type::make_range<::gos::temperature::type::Real>(0.0, 255.0);
}

namespace tune {
Tune k;
void time() {
  switch (gtv::pid::tune::time::unit) {
  case GOT_PI_TUNE_TIME_UNIT_MILLISECONDS:
    gt::pid::parameter.Time =
      static_cast<type::Real>(gtv::timing::interval);
    break;
  case GOT_PI_TUNE_TIME_UNIT_SECONDS:
  case GOT_PI_TUNE_TIME_UNIT_DEFAULT:
    gt::pid::parameter.Time =
      static_cast<type::Real>(gtv::timing::interval) / 1000.0;
    break;
  case GOT_PI_TUNE_TIME_UNIT_MINUTES:
    gt::pid::parameter.Time =
      static_cast<type::Real>(gtv::timing::interval) / 60000.0;
    break;
  }
}
} // namespace tune

} // namespace pid
} // namespace temperature
} // namespace gos
