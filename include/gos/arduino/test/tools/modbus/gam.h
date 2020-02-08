#ifndef GOS_ARDUINO_TEST_TOOL_MODBUS_GAM_SLAVE_H_
#define GOS_ARDUINO_TEST_TOOL_MODBUS_GAM_SLAVE_H_

#include <gos/arduino/test/tools/modbus/slave.h>
#include <gos/arduino/test/tools/arduino/stream.h>

#include <gatlmodbus.h>

namespace gos {
namespace arduino {
namespace test {
namespace tools {
namespace modbus {

typedef uint16_t Type;

typedef ::gos::atl::modbus::Handler<Type> Base;
typedef ::gos::atl::modbus::structures::Parameter<Type> Parameter;
typedef ::gos::atl::modbus::structures::Variable<Type> Variable;
typedef ::gos::atl::buffer::Holder<Type, char> Buffer;

class gam;

class Handler : public virtual Base {
  friend class gam;
public:
  Handler(const int& id, const size_t& size);
  MODBUS_TYPE_RESULT ReadCoils(
    const MODBUS_TYPE_FUNCTION& function,
    const Type& address,
    const Type& length);
  MODBUS_TYPE_RESULT ReadDiscreteInputs(
    const MODBUS_TYPE_FUNCTION& function,
    const Type& address,
    const Type& length);
  MODBUS_TYPE_RESULT ReadHoldingRegisters(
    const MODBUS_TYPE_FUNCTION& function,
    const Type& address,
    const Type& length);
  MODBUS_TYPE_RESULT ReadInputRegisters(
    const MODBUS_TYPE_FUNCTION& function,
    const Type& address,
    const Type& length);
  MODBUS_TYPE_RESULT WriteCoils(
    const MODBUS_TYPE_FUNCTION& function,
    const Type& address,
    const Type& length);
  MODBUS_TYPE_RESULT WriteHoldingRegisters(
    const MODBUS_TYPE_FUNCTION& function,
    const Type& address,
    const Type& length);
  MODBUS_TYPE_RESULT ReadExceptionStatus(
    const MODBUS_TYPE_FUNCTION& function);
protected:
  Parameter parameter_;
  Variable variable_;
  Buffer request_;
  Buffer response_;
};

class gam : public virtual slave {
public:
  gam(Stream& stream, const int& id, const size_t& size);
  void begin(const int& baud);
  void loop();
private:
  Stream& stream_;
  Handler handler_;
};

} // namespace modbus
} // namespace tools
} // namespace test
} // namespace arduino 
} // namespace gos

#endif
