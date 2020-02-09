#ifndef GOS_ARDUINO_SKETCHES_TESTBOARD_SIMPLE_MODBUS_VARIABLE_H_
#define GOS_ARDUINO_SKETCHES_TESTBOARD_SIMPLE_MODBUS_VARIABLE_H_

#include "type.h"

namespace gos {
namespace modbus {

namespace variables {
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
