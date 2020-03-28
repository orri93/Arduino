#include <gatlstring.h>

#include "variable.h"
#include "format.h"
#include "sensor.h"
#include "macro.h"
#include "text.h"
#include "type.h"

namespace gatl = ::gos::atl;

namespace gt = ::gos::temperature;
namespace gtv = ::gos::temperature::variables;
namespace gtfdb = ::gos::temperature::format::display::buffer;
namespace gtf = ::gos::temperature::format;

namespace gos {
namespace temperature {
namespace sensor {

gatl::sensor::Status Max6675Sensor::measure() {
  if (max6675.read(Value)) {
    Last = check();
  } else {
    Last = gatl::sensor::Status::Fault;
  }
  return Last;
}

const char* Max6675Sensor::error(uint8_t& length) {
  return max6675.error(length);
}

::gos::Max6675 max6675(PIN_MAX6675_CS);
Max6675Sensor max6675sensor;

void read() {
  max6675sensor.measure();
  gtv::temperature = static_cast<type::Real>(max6675sensor.Value);
#ifndef NO_DISPLAY
  switch (max6675sensor.Last) {
  case gatl::sensor::Status::Operational:
    gatl::format::real<type::Real, uint8_t>(
      gtfdb::first,
      gtv::temperature,
      gtf::real::option,
      &gtfdb::text::temperature,
      &gtfdb::text::unit::degree::centigrade);
    break;
  case gatl::sensor::Status::BelowRange:
    gatl::format::real<type::Real, uint8_t>(
      gtfdb::first,
      gtv::temperature,
      gtf::real::option,
      &gtfdb::text::temperature,
      &gtfdb::text::codes::belowrange);
    break;
  case gatl::sensor::Status::AboveRange:
    gatl::format::real<type::Real, uint8_t>(
      gtfdb::first,
      gtv::temperature,
      gtf::real::option,
      &gtfdb::text::temperature,
      &gtfdb::text::codes::aboverange);
    break;
  case gatl::sensor::Status::Fault:
    gatl::string::copy(gtfdb::first, GOS_TCT_SENSOR_FAULT);
    break;
  }
#endif
}

} // namespace sensor
} // namespace temperature
} // namespace gos
