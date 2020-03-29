#ifndef GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_FORMAT_H_
#define GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_FORMAT_H_

#include <gatlbuffer.h>
#include <gatlformat.h>

namespace gos {
namespace temperature {

namespace format {
namespace display {
namespace buffer {
typedef ::gos::atl::buffer::Holder<> Holder;
extern Holder first;
extern Holder second;
namespace text {
extern Holder interval;
extern Holder manual;
extern Holder setpoint;
extern Holder temperature;
extern Holder kp;
extern Holder ki;
extern Holder kd;
extern Holder ti;
extern Holder td;
extern Holder minsens;
extern Holder maxsens;
namespace unit {
namespace degree {
extern Holder centigrade;
} // namespace degree
} // namespace unit
namespace codes {
extern Holder belowrange;
extern Holder aboverange;
} // namespace codes
} // namespace text
} // namespace buffer
} // namespace display

namespace real {
extern ::gos::atl::format::option::Number option;
}

void initialize();

} // namespace format

} // namespace temperature
} // namespace gos

#endif
