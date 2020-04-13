#include "variable.h"
#include "macro.h"
#include "value.h"

namespace gt = ::gos::temperature;

namespace gos {
namespace temperature {
namespace variables {

type::Status status = type::Status::idle;

type::Unsigned force = GOT_PI_FORCE_OFF;

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

namespace pid {
namespace tune {
namespace time {
type::Unsigned unit = GOT_PI_TUNE_TIME_UNIT_DEFAULT;
}
}
}

namespace temporary {
bool boolean = false;
uint8_t byte = 0;
type::Unsigned integer = gt::value::zero::Unsigned;
type::Real real = gt::value::zero::Real;
} // namespace temporary

} // namespace variables
} // namespace temperature
} // namespace gos