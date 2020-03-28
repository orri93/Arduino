#include "variable.h"
#include "macro.h"
#include "value.h"

namespace gt = ::gos::temperature;

namespace gos {
namespace temperature {
namespace variables {

type::Unsigned output = gt::value::zero::Unsigned;
type::Real temperature = gt::value::zero::Real;

namespace timing {
unsigned long tick;
unsigned long next = 0;
type::Unsigned interval = DEFAULT_INTERVAL;
}

namespace controller {
type::Unsigned manual = gt::value::zero::Unsigned;
}

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