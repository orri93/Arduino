#ifndef GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_SENSOR_H_
#define GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_SENSOR_H_

#include <gosmax6675.h>
#include <gatlsensor.h>

namespace gos {
namespace temperature {
namespace sensor {

class Max6675Sensor : virtual public ::gos::atl::Sensor<double, uint8_t> {
public:
  ::gos::atl::sensor::Status measure();
  const char* error(uint8_t& length);
};

extern ::gos::Max6675 max6675;
extern Max6675Sensor max6675sensor;

void read();

} // namespace sensor
} // namespace temperature
} // namespace gos

#endif

