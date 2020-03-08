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
}

} // namespace retrieve

} // namespace eeprom
} // namespace temperature
} // namespace gos