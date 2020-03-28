#include <gatlsensor.h>

#include "sensor.h"
#include "macro.h"

namespace gatl = ::gos::atl;

namespace gos {
namespace temperature {
namespace sensor {

Max6675Sensor::Max6675Sensor() {
  Range.lowest = 0.0;
  Range.highest = 500.0;
}

Max6675Sensor::Status Max6675Sensor::measure() {
  error_ = nullptr;
  if (max6675.read(Value)) {
    Last = check();
  } else {
    error_ = 
    Last = Status::Fault;
  }
  return Last;
}

const char* Max6675Sensor::error(uint8_t& length) {
  return max6675.error(length);
}

::gos::Max6675 max6675(PIN_MAX6675_CS);
Max6675Sensor max6675sensor;

void read() {

}

} // namespace sensor
} // namespace temperature
} // namespace gos
