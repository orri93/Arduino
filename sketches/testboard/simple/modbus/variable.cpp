#include "variable.h"
#include "value.h"

namespace gos {
namespace modbus {

namespace variables {
uint8_t coils = 0;
namespace led {
namespace red {
namespace last {
bool a = false;
bool b = false;
} // namespace last
bool a = false;
bool b = false;
} // namespace red
namespace blue {
namespace last {
uint8_t a = 0;
uint8_t b = 0;
} // namespace last
uint8_t a = 0;
uint8_t b = 0;
} // namespace red
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
