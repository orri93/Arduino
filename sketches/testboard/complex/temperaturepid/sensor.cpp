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

#if defined(GOS_TEMPERATUREPID_SENSOR_DS18B20)
SensorDs18b20::SensorDs18b20() : onewire_(nullptr), dallas_(nullptr) {
}

void SensorDs18b20::begin() {
  onewire_ = new OneWire(PIN_DS18B20);
  dallas_ = new DallasTemperature(onewire_);
  dallas_->begin();
}

gatl::sensor::Status SensorDs18b20::measure() {
  dallas_->requestTemperatures();
  Value = dallas_->getTempCByIndex(0);
  return (Last = check());
}

const char* SensorDs18b20::error(uint8_t& length) {
  length = 0;
  return nullptr;
}

SensorDs18b20 temperature;

#else
SensorMax6675::SensorMax6675() : max6675_(nullptr) {
}

void SensorMax6675::begin() {
  SPI.begin();
  max6675_ = new ::gos::Max6675(PIN_MAX6675_CS);
  max6675_->initialize();
}

gatl::sensor::Status SensorMax6675::measure() {
  if (max6675_->read(Value)) {
    if ((!isnan(Value)) && (!isinf(Value))) {
      return (Last = check());
    } 
  } 
  return Last = gatl::sensor::Status::Fault;
}

const char* SensorMax6675::error(uint8_t& length) {
  return max6675_->error(length);
}

SensorMax6675 temperature;
#endif

gatl::sensor::Status read() {
  gatl::sensor::Status result = temperature.measure();
  gtv::temperature = static_cast<type::Real>(temperature.Value);
#ifndef NO_DISPLAY
  switch (result) {
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
    gt::sensor::error::message = temperature.error(gt::sensor::error::length);
    if (gt::sensor::error::length > 0) {
      gatl::format::message(
        gtfdb::first,
        gt::sensor::error::message,
        gt::sensor::error::length,
        &gtfdb::text::temperature);
    } else {
      gatl::format::message(
        gtfdb::first,
        GOS_TCT_FAULT,
        sizeof(GOS_TCT_FAULT),
        &gtfdb::text::temperature);
    }
    break;
  }
#endif
  return result;
}

namespace error {
const char* message = nullptr;
uint8_t length = 0;
}

} // namespace sensor
} // namespace temperature
} // namespace gos
