#include <EEPROM.h>

#include "pid.h"
#include "eeprom.h"
#include "variable.h"
#include "sensor.h"
#include "value.h"

#define GOS_T_IEV(i,v) EEPROM.get<type::Real>(i, v); \
if (isnan(v) || isinf(v)) { v = 0.0; EEPROM.put<type::Real>(i, v); }

namespace gatl = ::gos::atl;
namespace gatlp = ::gos::atl::pid;
namespace gatlpt = ::gos::atl::pid::time;

namespace gt = ::gos::temperature;
namespace gtv = ::gos::temperature::variables;
namespace gtvc = ::gos::temperature::variables::controller;
namespace gtvi = ::gos::temperature::variables::timing;
namespace gtval = ::gos::temperature::value;
namespace gtvptt = ::gos::temperature::variables::pid::tune::time;
namespace gtp = ::gos::temperature::pid;
namespace gts = ::gos::temperature::sensor;

namespace gos {
namespace temperature {
namespace eeprom {

namespace retrieve {

void initial() {
  gt::variables::modbus::coils = EEPROM.read(GOS_TC_EEPROM_INDEX_COILS);
  EEPROM.get<type::Unsigned>(GOS_TC_EEPROM_INDEX_INTERVAL, gtvi::interval);
  EEPROM.get<type::Unsigned>(GOS_TC_EEPROM_INDEX_MANUAL, gtvc::manual);
  GOS_T_IEV(GOS_TC_EEPROM_INDEX_SETPOINT, gtp::parameter.Setpoint);
  GOS_T_IEV(GOS_TC_EEPROM_INDEX_KP, gtp::parameter.Kp);
  if (bitRead(gtv::modbus::coils, GOS_TCV_COIL_BIT_TUNE_TIME_MASTER)) {
    GOS_T_IEV(GOS_TC_EEPROM_INDEX_KITI, gtp::tune::t.Ti);
    GOS_T_IEV(GOS_TC_EEPROM_INDEX_KDTD, gtp::tune::t.Td);
  } else {
    GOS_T_IEV(GOS_TC_EEPROM_INDEX_KITI, gtp::tune::k.Ki);
    GOS_T_IEV(GOS_TC_EEPROM_INDEX_KDTD, gtp::tune::k.Kd);
  }
  GOS_T_IEV(GOS_TC_EEPROM_INDEX_MIN_SENS, gt::sensor::temperature.Range.lowest);
  GOS_T_IEV(GOS_TC_EEPROM_INDEX_MAX_SENS, gt::sensor::temperature.Range.highest);
  EEPROM.get<type::Unsigned>(GOS_TC_EEPROM_INDEX_TIME_TUNE, gtvptt::unit);

  if (gtvi::interval == 0) {
    gtvi::interval = gtval::defaultval::timing::Interval;
    EEPROM.put<type::Unsigned>(GOS_TC_EEPROM_INDEX_INTERVAL, gtvi::interval);
  }
  if (gts::temperature.Range.highest <= gts::temperature.Range.lowest) {
    gts::temperature.Range.lowest = gtval::zero::Real;
    gts::temperature.Range.highest = gtval::defaultval::MaxSensorRange;
    EEPROM.put<type::Unsigned>(
      GOS_TC_EEPROM_INDEX_MIN_SENS,
      gts::temperature.Range.lowest);
    EEPROM.put<type::Unsigned>(
      GOS_TC_EEPROM_INDEX_MAX_SENS,
      gts::temperature.Range.highest);
  }
  gtv::temporary::boolean = false;
  gtv::temporary::byte = highByte(gtvptt::unit);
  if (gtv::temporary::byte > GOT_PI_TUNE_TIME_UNIT_MAXIMUM) {
    gtvptt::unit &= 0xff00;
    gtv::temporary::boolean = true;
  }
  gtv::temporary::byte = lowByte(gtvptt::unit);
  if (gtv::temporary::byte > GOT_PI_TUNE_TIME_UNIT_MAXIMUM) {
    gtvptt::unit &= 0x00ff;
    gtv::temporary::boolean = true;
  }
  if (gtv::temporary::boolean) {
    EEPROM.put<type::Unsigned>(GOS_TC_EEPROM_INDEX_TIME_TUNE, gtvptt::unit);
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