#ifndef GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_SENSOR_H_
#define GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_SENSOR_H_

#if defined(GOS_TEMPERATUREPID_SENSOR_DS18B20)
#include <OneWire.h> 
#include <DallasTemperature.h>
#elif defined(GOS_TEMPERATUREPID_SENSOR_MAX31865)
#include <SPI.h>
#include <gosmax31865.h>
#else
#include <SPI.h>
#include <gosmax6675.h>
#endif
#include <gatlsensor.h>

namespace gos {
namespace temperature {
namespace sensor {

typedef ::gos::atl::Sensor<double, uint8_t> TypedSensor;

#if defined(GOS_TEMPERATUREPID_SENSOR_DS18B20)

class SensorDs18b20 : public TypedSensor {
public:
  SensorDs18b20();
  void begin();
  ::gos::atl::sensor::Status measure();
  const char* error(uint8_t& length);
private:
  OneWire* onewire_;
  DallasTemperature* dallas_;
};
extern SensorDs18b20 temperature;

#elif defined(GOS_TEMPERATUREPID_SENSOR_MAX31865)

class SensorMax31865 : public TypedSensor {
public:
  SensorMax31865();
  void begin();
  ::gos::atl::sensor::Status measure();
  const char* error(uint8_t& length);
private:
  ::gos::Max31865* max31865_;
};
extern SensorMax31865 temperature;

#else

class SensorMax6675 : public TypedSensor {
public:
  SensorMax6675();
  void begin();
  ::gos::atl::sensor::Status measure();
  const char* error(uint8_t& length);
private:
  ::gos::Max6675* max6675_;
};
extern SensorMax6675 temperature;

#endif

::gos::atl::sensor::Status read();
void report();

namespace error {
extern const char* message;
extern uint8_t length;
}

} // namespace sensor
} // namespace temperature
} // namespace gos

#endif

