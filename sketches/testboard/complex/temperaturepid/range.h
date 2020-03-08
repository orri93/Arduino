#ifndef GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_RANGE_H_
#define GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_RANGE_H_

#include <gatltype.h>

#include "type.h"

namespace gos {
namespace temperature {
namespace range {

typedef ::gos::atl::type::range<type::Real> Real;
typedef ::gos::atl::type::range<type::Unsigned> Unsigned;

extern Real setpoint;
extern Unsigned output;

} // namespace range
} // namespace temperature
} // namespace gos

#endif
