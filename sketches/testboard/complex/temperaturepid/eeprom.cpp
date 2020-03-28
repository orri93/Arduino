#include <EEPROM.h>

#include "pid.h"
#include "eeprom.h"
#include "variable.h"

namespace gt = ::gos::temperature;
namespace gtv = ::gos::temperature::variables;
namespace gtvc = ::gos::temperature::variables::controller;
namespace gtvi = ::gos::temperature::variables::timing;
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
#ifdef PID_STORE_TIME_TUNE
  EEPROM.get<type::Real>(GOS_TC_EEPROM_INDEX_KITI, gtp::tune::t.Ti);
  EEPROM.get<type::Real>(GOS_TC_EEPROM_INDEX_KDTD, gtp::tune::t.Td);
#else
  EEPROM.get<type::Real>(GOS_TC_EEPROM_INDEX_KITI, gtp::tune::k.Ki);
  EEPROM.get<type::Real>(GOS_TC_EEPROM_INDEX_KDTD, gtp::tune::k.Kd);
#endif

  if (gtvi::interval == 0) {
    gtvi::interval = 1000;
    EEPROM.put<type::Unsigned>(GOS_TC_EEPROM_INDEX_INTERVAL, gtvi::interval);
  }
}

} // namespace retrieve

} // namespace eeprom
} // namespace temperature
} // namespace gos