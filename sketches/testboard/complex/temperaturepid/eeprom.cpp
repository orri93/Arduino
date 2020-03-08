#include <EEPROM.h>

#include "pid.h"
#include "eeprom.h"
#include "variable.h"

namespace gt = ::gos::temperature;

namespace gos {
namespace temperature {
namespace eeprom {

namespace retrieve {

void initial() {
  gt::variables::modbus::coils = EEPROM.read(GOS_TC_EEPROM_INDEX_COILS);
  EEPROM.get(GOS_TC_EEPROM_INDEX_INTERVAL, gt::variables::interval);
  EEPROM.get(GOS_TC_EEPROM_INDEX_MANUAL, gt::variables::manual);
  EEPROM.get(GOS_TC_EEPROM_INDEX_SETPOINT, gt::pid::parameter.Setpoint);
  EEPROM.get(GOS_TC_EEPROM_INDEX_KP, gt::pid::parameter.Kp);
#ifdef PID_STORE_TIME_TUNE
  EEPROM.get(GOS_TC_EEPROM_INDEX_KITI, gt::pid::tune::t.Ti);
  EEPROM.get(GOS_TC_EEPROM_INDEX_KDTD, gt::pid::tune::t.Td);
#else
  EEPROM.get(GOS_TC_EEPROM_INDEX_KITI, gt::pid::tune::k.Ki);
  EEPROM.get(GOS_TC_EEPROM_INDEX_KDTD, gt::pid::tune::k.Kd);
#endif
}

} // namespace retrieve

} // namespace eeprom
} // namespace temperature
} // namespace gos