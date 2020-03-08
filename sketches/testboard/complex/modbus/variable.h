#ifndef GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_MODBUS_VARIABLE_H_
#define GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_MODBUS_VARIABLE_H_

#include "type.h"

namespace gos {
namespace modbus {

namespace variables {
extern uint8_t coils;
namespace led {
namespace red {
extern bool last;
extern bool value;
} // namespace red
namespace blue {
extern uint8_t last;
extern uint8_t value;
} // namespace blue
} // namespace led
namespace real {
extern type::Real first;
extern type::Real second;
} // namespace real
extern type::Signed signedv;
extern type::Unsigned unsignedv;
namespace temporary {
extern type::Unsigned integer;
extern type::Real real;
} // namespace temporary
} // namespace variables

} // namespace modbus
} // namespace gos

#endif
