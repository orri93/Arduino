#include "variable.h"
#include "value.h"

namespace gos {
namespace modbus {

namespace variables {
uint8_t coils = 0;
namespace led {
namespace red {
bool last = false;
bool value = false;
} // namespace red
namespace blue {
uint8_t last = 0;
uint8_t value = 0;
} // namespace blue
} // namespace led
namespace real {
type::Real first = value::zero::Real;
type::Real second = value::zero::Real;
} // namespace real
type::Signed signedv = value::zero::Signed;
type::Unsigned unsignedv = value::zero::Unsigned;
namespace temporary {
type::Unsigned integer;
type::Real real;
} // namespace temporary
} // namespace variables

} // namespace modbus
} // namespace gos
