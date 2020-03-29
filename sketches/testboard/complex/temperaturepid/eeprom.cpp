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
    gtp::tune::k.Ki = gatlpt::minutes::Ki(gtp::parameter.Kp, gtp::tune::t.Ti);
    gtp::tune::k.Kd = gatlpt::minutes::Kd(gtp::parameter.Kp, gtp::tune::t.Td);
  } else {
    EEPROM.get<type::Real>(GOS_TC_EEPROM_INDEX_KITI, gtp::tune::k.Ki);
    EEPROM.get<type::Real>(GOS_TC_EEPROM_INDEX_KDTD, gtp::tune::k.Kd);
    gtp::tune::t.Ti = gatlpt::minutes::Ti(gtp::parameter.Kp, gtp::tune::k.Ki);
    gtp::tune::t.Td = gatlpt::minutes::Td(gtp::parameter.Kp, gtp::tune::k.Kd);
  }
  EEPROM.get<double>(
    GOS_TC_EEPROM_INDEX_MIN_SENS,
    gt::sensor::max6675sensor.Range.lowest);
  EEPROM.get<double>(
    GOS_TC_EEPROM_INDEX_MAX_SENS,
    gt::sensor::max6675sensor.Range.highest);

  if (gtvi::interval == 0) {
    gtvi::interval = gtval::defaultval::timing::Interval;
    EEPROM.put<type::Unsigned>(GOS_TC_EEPROM_INDEX_INTERVAL, gtvi::interval);
  }
  if (gt::sensor::max6675sensor.Range.highest <= 0.0) {
    gt::sensor::max6675sensor.Range.highest = gtval::defaultval::MaxSensorRange;
    EEPROM.put<type::Unsigned>(
      GOS_TC_EEPROM_INDEX_MAX_SENS,
      gt::sensor::max6675sensor.Range.highest);
  }

  gt::pid::parameter.PonE = bitRead(gtv::modbus::coils, GOS_TCV_COIL_BIT_PONE);
}

} // namespace retrieve

} // namespace eeprom
} // namespace temperature
} // namespace gos