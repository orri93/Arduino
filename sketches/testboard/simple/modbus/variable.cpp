#include "variable.h"
#include "value.h"

namespace gos {
namespace modbus {

namespace variables {
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
