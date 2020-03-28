#ifndef GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_VALUE_H_
#define GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_VALUE_H_

#include "type.h"

namespace gos {
namespace temperature {

namespace value {
namespace zero {
extern const type::Real Real;
extern const type::Signed Signed;
extern const type::Unsigned Unsigned;
} // namespace zero
namespace defaultval {
namespace timing {
extern const type::Unsigned Interval;
} // namespace timing
extern const double MaxSensorRange;
} // namespace defaultval
extern const type::Unsigned MaxManual;
} // namespace value

} // namespace temperature
} // namespace gos

#endif
