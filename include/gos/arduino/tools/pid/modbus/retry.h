#ifndef GOS_ARDUINO_TOOL_PID_MODBUS_MASTER_RETRY_H_
#define GOS_ARDUINO_TOOL_PID_MODBUS_MASTER_RETRY_H_

#include <gos/arduino/tools/pid/modbus/master.h>

namespace gos {
namespace arduino {
namespace tools {
namespace pid {
namespace modbus {
namespace master {
namespace retry {

void set(const int& retry);

::gos::arduino::tools::pid::modbus::types::result connect();

namespace read {
::gos::arduino::tools::pid::modbus::types::result input(
  ::gos::arduino::tools::pid::types::registry::Input& input);
::gos::arduino::tools::pid::modbus::types::result holding(
  ::gos::arduino::tools::pid::types::registry::Holding& holding);
} // namespace read
namespace write {
::gos::arduino::tools::pid::modbus::types::result holding(
  const ::gos::arduino::tools::pid::types::registry::Holding& holding);
::gos::arduino::tools::pid::modbus::types::result interval(
  const ::gos::arduino::tools::pid::types::Unsigned& interval);
::gos::arduino::tools::pid::modbus::types::result manual(
  const ::gos::arduino::tools::pid::types::Unsigned& manual);
::gos::arduino::tools::pid::modbus::types::result setpoint(
  const ::gos::arduino::tools::pid::types::Real& setpoint);
::gos::arduino::tools::pid::modbus::types::result kp(
  const ::gos::arduino::tools::pid::types::Real& kp);
::gos::arduino::tools::pid::modbus::types::result ki(
  const ::gos::arduino::tools::pid::types::Real& ki);
::gos::arduino::tools::pid::modbus::types::result kd(
  const ::gos::arduino::tools::pid::types::Real& kd);
::gos::arduino::tools::pid::modbus::types::result tuning(
  const ::gos::arduino::tools::pid::types::Real& kp,
  const ::gos::arduino::tools::pid::types::Real& ki);
::gos::arduino::tools::pid::modbus::types::result tuning(
  const ::gos::arduino::tools::pid::types::Real& kp,
  const ::gos::arduino::tools::pid::types::Real& ki,
  const ::gos::arduino::tools::pid::types::Real& kd);
::gos::arduino::tools::pid::modbus::types::result sensor(
  const ::gos::arduino::tools::pid::types::Real& minimum,
  const ::gos::arduino::tools::pid::types::Real& maximum);
::gos::arduino::tools::pid::modbus::types::result time(
  const ::gos::arduino::tools::pid::types::Unsigned& tuning);
::gos::arduino::tools::pid::modbus::types::result force(
  const ::gos::arduino::tools::pid::types::Unsigned& force);
} // namespace write

} // namespace retry
} // namespace master
} // namespace modbus
} // namespace pid
} // namespace tools
} // namespace arduino 
} // namespace gos

#endif
