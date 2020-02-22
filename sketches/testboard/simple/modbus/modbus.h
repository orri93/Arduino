#ifndef GOS_ARDUINO_SKETCHES_TESTBOARD_SIMPLE_MODBUS_H_
#define GOS_ARDUINO_SKETCHES_TESTBOARD_SIMPLE_MODBUS_H_

#include <gatlmodbus.h>

#include "macro.h"

namespace gos {
namespace modbus {

#ifdef MODBUS_HANDLER_INTERFACE
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
#else
namespace callback {
namespace read {
MODBUS_TYPE_RESULT coils(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length);
namespace discrete {
MODBUS_TYPE_RESULT inputs(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length);
}
namespace holding {
MODBUS_TYPE_RESULT registers(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length);
}
namespace input {
MODBUS_TYPE_RESULT registers(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length);
}
namespace exception {
MODBUS_TYPE_RESULT status();
}
}
namespace write {
MODBUS_TYPE_RESULT coils(
  const MODBUS_TYPE_FUNCTION& function,
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length);
namespace holding {
MODBUS_TYPE_RESULT registers(
  const MODBUS_TYPE_FUNCTION& function,
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length);
}
}
}
#endif

void initialize();

namespace buffer {
typedef ::gos::atl::buffer::Holder<uint16_t, MODBUS_TYPE_BUFFER> Holder;
extern Holder request;
extern Holder response;
} // namespace buffer

typedef ::gos::atl::modbus::structures::Parameter<> Parameter;
typedef ::gos::atl::modbus::structures::Variable<> Variable;

extern Parameter parameter;
extern Variable variable;

} // namespace modbus
} // namespace gos

#endif
