#ifndef GOS_ARDUINO_TEST_TOOL_MODBUS_GAM_SLAVE_H_
#define GOS_ARDUINO_TEST_TOOL_MODBUS_GAM_SLAVE_H_

#include <random>

#include <gos/arduino/test/tools/modbus/slave.h>
#include <gos/arduino/test/tools/arduino/stream.h>
#include <gos/arduino/test/tools/memory.h>

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
typedef ::gos::atl::buffer::Holder<Type, MODBUS_TYPE_BUFFER> Buffer;
typedef ::gos::arduino::test::tools::Memory<MODBUS_TYPE_BUFFER> Memory;

class gam;

class Handler : public virtual Base {
  friend class gam;
public:
  Handler(const int& id, const size_t& size);
  bool create();
  MODBUS_TYPE_RESULT ReadCoils(
    const Type& address,
    const Type& length);
  MODBUS_TYPE_RESULT ReadDiscreteInputs(
    const Type& address,
    const Type& length);
  MODBUS_TYPE_RESULT ReadHoldingRegisters(
    const Type& address,
    const Type& length);
  MODBUS_TYPE_RESULT ReadInputRegisters(
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
  MODBUS_TYPE_RESULT ReadExceptionStatus();
protected:
  typedef std::random_device Device;
  typedef std::default_random_engine Engine;
  typedef  std::uniform_int_distribution<unsigned short> Distribution;

  Device random_;
  Engine engine_;
  Distribution distribution_;

  Memory memory_;
  Parameter parameter_;
  Variable variable_;
  Buffer request_;
  Buffer response_;
  uint8_t coils_;
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
