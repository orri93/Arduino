#ifndef GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_PID_H_
#define GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_PID_H_

#include <gatlpid2.h>

#include "type.h"

namespace gos {
namespace temperature {
namespace pid {

typedef ::gos::atl::pid::wiki::Parameter<
  ::gos::temperature::type::Real,
  ::gos::temperature::type::Real,
  ::gos::temperature::type::Real> Parameter;
typedef ::gos::atl::pid::wiki::Variable<
  ::gos::temperature::type::Real> Variable;

extern Parameter parameter;
extern Variable variable;

void create();

namespace tune {
typedef ::gos::atl::pid::Tune<::gos::temperature::type::Real> Tune;
extern Tune k;
void time();
} // namespace tune

} // namespace pid
} // namespace temperature
} // namespace gos

#endif
