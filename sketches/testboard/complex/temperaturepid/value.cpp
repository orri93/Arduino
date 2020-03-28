#include "value.h"

namespace gos {
namespace temperature {

namespace value {
namespace zero {
const type::Real Real = 0.0F;
const type::Signed Signed = 0;
const type::Unsigned Unsigned = 0;
} // namespace zero
namespace defaultval {
namespace timing {
const type::Unsigned Interval = 2000;
} // namespace timing
const double MaxSensorRange = 500.0;
} // namespace defaultval
const type::Unsigned MaxManual = 255;
} // namespace value

} // namespace temperature
} // namespace gos
