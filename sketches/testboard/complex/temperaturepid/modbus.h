#ifndef GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_MODBUS_H_
#define GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_MODBUS_H_

#include <gatlmodbus.h>

#include "macro.h"

namespace gos {
namespace temperature {
namespace modbus {

class Handler : public ::gos::atl::modbus::Handler<MODBUS_TYPE_DEFAULT> {
public:
  MODBUS_TYPE_RESULT ReadCoils(
    const MODBUS_TYPE_DEFAULT& address,
    const MODBUS_TYPE_DEFAULT& length);
  MODBUS_TYPE_RESULT ReadDiscreteInputs(
    const MODBUS_TYPE_DEFAULT& address,
    const MODBUS_TYPE_DEFAULT& length);
  MODBUS_TYPE_RESULT ReadHoldingRegisters(
    const MODBUS_TYPE_DEFAULT& address,
    const MODBUS_TYPE_DEFAULT& length);
  MODBUS_TYPE_RESULT ReadInputRegisters(
    const MODBUS_TYPE_DEFAULT& address,
    const MODBUS_TYPE_DEFAULT& length);
  MODBUS_TYPE_RESULT WriteCoils(
    const MODBUS_TYPE_FUNCTION& function,
    const MODBUS_TYPE_DEFAULT& address,
    const MODBUS_TYPE_DEFAULT& length);
  MODBUS_TYPE_RESULT WriteHoldingRegisters(
    const MODBUS_TYPE_FUNCTION& function,
    const MODBUS_TYPE_DEFAULT& address,
    const MODBUS_TYPE_DEFAULT& length);
  MODBUS_TYPE_RESULT ReadExceptionStatus();
};
extern Handler handler;

namespace buffer {
typedef ::gos::atl::buffer::Holder<uint16_t, MODBUS_TYPE_BUFFER> Holder;
extern Holder request;
extern Holder response;
} // namespace buffer
typedef ::gos::atl::modbus::structures::Parameter<> Parameter;
typedef ::gos::atl::modbus::structures::Variable<> Variable;
extern Parameter parameter;
extern Variable variable;

void initialize();

} // namespace modbus
} // namespace temperature
} // namespace gos

#endif
