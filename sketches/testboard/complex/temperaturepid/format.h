#ifndef GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_FORMAT_H_
#define GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_FORMAT_H_

#include <gatlbuffer.h>

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
} // namespace text
} // namespace buffer
} // namespace display
} // namespace format

} // namespace temperature
} // namespace gos

#endif
