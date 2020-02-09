#ifndef GOS_ARDUINO_SKETCHES_TESTBOARD_SIMPLE_MODBUS_FORMAT_H_
#define GOS_ARDUINO_SKETCHES_TESTBOARD_SIMPLE_MODBUS_FORMAT_H_

#include <gatlbuffer.h>

namespace gos {
namespace modbus {

namespace format {
namespace display {
namespace buffer {
typedef ::gos::atl::buffer::Holder<> Holder;
extern Holder first;
extern Holder second;
} // namespace buffer
namespace crc {
namespace last {
extern uint16_t first;
extern uint16_t second;
} // namespace last
} // namespace crc
} // namespace display
} // namespace format

} // namespace modbus
} // namespace gos

#endif
