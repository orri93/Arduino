#ifndef GOS_ARDUINO_TEST_TOOL_MODBUS_GAM_SLAVE_H_
#define GOS_ARDUINO_TEST_TOOL_MODBUS_GAM_SLAVE_H_

#include <memory>
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

typedef ::gos::atl::modbus::structures::Parameter<Type> Parameter;
typedef ::gos::atl::modbus::structures::Variable<Type> Variable;
typedef ::gos::atl::buffer::Holder<Type, MODBUS_TYPE_BUFFER> Buffer;
typedef ::gos::arduino::test::tools::Memory<MODBUS_TYPE_BUFFER> Memory;

class gam;

#ifdef MODBUS_HANDLER_INTERFACE
typedef ::gos::atl::modbus::Handler<Type> Base;
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
#else
typedef std::random_device Device;
typedef std::default_random_engine Engine;
typedef std::uniform_int_distribution<unsigned short> Distribution;

typedef std::unique_ptr<Buffer> BufferPointer;

extern Device _random;
extern Engine _engine;
extern Distribution _distribution;

extern Memory _memory;
extern Parameter _parameter;
extern Variable _variable;
extern BufferPointer _request;
extern BufferPointer _response;
extern uint8_t _coils;

bool initialize(const int& id, const size_t& size);

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

class gam : public virtual slave {
public:
  gam(Stream& stream, const int& id, const size_t& size);
  void begin(const int& baud);
  void loop();
private:
  Stream& stream_;
#ifdef MODBUS_HANDLER_INTERFACE
  Handler handler_;
#else
  int id_;
  size_t size_;
#endif
};

} // namespace modbus
} // namespace tools
} // namespace test
} // namespace arduino 
} // namespace gos

#endif
