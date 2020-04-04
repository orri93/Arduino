#ifndef GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_PID_H_
#define GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_PID_H_

#include <gatlpid.h>

#include "type.h"

namespace gos {
namespace temperature {
namespace pid {

typedef ::gos::atl::pid::Parameter<
  ::gos::temperature::type::Real,
  ::gos::temperature::type::Unsigned,
  ::gos::temperature::type::Real> Parameter;
typedef ::gos::atl::pid::Variable<::gos::temperature::type::Real> Variable;

extern Parameter parameter;
extern Variable variable;

void create();
void initialize();

namespace tune {
typedef ::gos::atl::pid::Tune<::gos::temperature::type::Real> Tune;
typedef ::gos::atl::pid::TimeTune<::gos::temperature::type::Real> TimeTune;
extern Tune k;
extern TimeTune t;
void calculate();
void tunings();
void time();
} // namespace tune

} // namespace pid
} // namespace temperature
} // namespace gos

#endif
