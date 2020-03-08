#include "variable.h"
#include "macro.h"
#include "value.h"

namespace gt = ::gos::temperature;

namespace gos {
namespace temperature {
namespace variables {

type::Real temperature = gt::value::zero::Real;
type::Real output = gt::value::zero::Real;

type::Unsigned interval = DEFAULT_INTERVAL;
type::Unsigned manual = gt::value::zero::Unsigned;

namespace modbus {
uint8_t coils = 0;
} // namespace modbus

namespace temporary {
bool boolean;
type::Unsigned integer;
type::Real real;
} // namespace temporary

} // namespace variables
} // namespace temperature
} // namespace gos