#include <EEPROM.h>

#include <gatlpid.h>

#include "pid.h"
#include "eeprom.h"
#include "variable.h"
#include "sensor.h"
#include "value.h"

namespace gatl = ::gos::atl;
namespace gatlp = ::gos::atl::pid;
namespace gatlpt = ::gos::atl::pid::time;

namespace gt = ::gos::temperature;
namespace gtv = ::gos::temperature::variables;
namespace gtvc = ::gos::temperature::variables::controller;
namespace gtvi = ::gos::temperature::variables::timing;
namespace gtval = ::gos::temperature::value;
namespace gtp = ::gos::temperature::pid;

namespace gos {
namespace temperature {
namespace eeprom {

namespace retrieve {

void initial() {
  gt::variables::modbus::coils = EEPROM.read(GOS_TC_EEPROM_INDEX_COILS);
  EEPROM.get<type::Unsigned>(GOS_TC_EEPROM_INDEX_INTERVAL, gtvi::interval);
  EEPROM.get<type::Unsigned>(GOS_TC_EEPROM_INDEX_MANUAL, gtvc::manual);
  EEPROM.get<type::Real>(GOS_TC_EEPROM_INDEX_SETPOINT, gtp::parameter.Setpoint);
  EEPROM.get<type::Real>(GOS_TC_EEPROM_INDEX_KP, gtp::parameter.Kp);
  if (bitRead(gtv::modbus::coils, GOS_TCV_COIL_BIT_TUNE_TIME_MASTER)) {
    EEPROM.get<type::Real>(GOS_TC_EEPROM_INDEX_KITI, gtp::tune::t.Ti);
    EEPROM.get<type::Real>(GOS_TC_EEPROM_INDEX_KDTD, gtp::tune::t.Td);
  } else {
    EEPROM.get<type::Real>(GOS_TC_EEPROM_INDEX_KITI, gtp::tune::k.Ki);
    EEPROM.get<type::Real>(GOS_TC_EEPROM_INDEX_KDTD, gtp::tune::k.Kd);
  }
  EEPROM.get<double>(
    GOS_TC_EEPROM_INDEX_MIN_SENS,
    gt::sensor::temperature.Range.lowest);
  EEPROM.get<double>(
    GOS_TC_EEPROM_INDEX_MAX_SENS,
    gt::sensor::temperature.Range.highest);
  EEPROM.get<type::Unsigned>(
    GOS_TC_EEPROM_INDEX_TIME_TUNE,
    gtv::pid::tune::time::unit);

  if (gtvi::interval == 0) {
    gtvi::interval = gtval::defaultval::timing::Interval;
    EEPROM.put<type::Unsigned>(GOS_TC_EEPROM_INDEX_INTERVAL, gtvi::interval);
  }
  if (gt::sensor::temperature.Range.highest <= 0.0) {
    gt::sensor::temperature.Range.highest = gtval::defaultval::MaxSensorRange;
    EEPROM.put<type::Unsigned>(
      GOS_TC_EEPROM_INDEX_MAX_SENS,
      gt::sensor::temperature.Range.highest);
  }
}

} // namespace retrieve

namespace tune {
void restore() {
  if (bitRead(gtv::modbus::coils, GOS_TCV_COIL_BIT_TUNE_TIME_MASTER)) {
    EEPROM.put<type::Real>(GOS_TC_EEPROM_INDEX_KITI, gtp::tune::t.Ti);
    EEPROM.put<type::Real>(GOS_TC_EEPROM_INDEX_KDTD, gtp::tune::t.Td);
  } else {
    EEPROM.put<type::Real>(GOS_TC_EEPROM_INDEX_KITI, gtp::tune::k.Ki);
    EEPROM.put<type::Real>(GOS_TC_EEPROM_INDEX_KDTD, gtp::tune::k.Kd);
  }
}
} // namespace tune

} // namespace eeprom
} // namespace temperature
} // namespace gos