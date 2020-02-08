#include <iostream>

#include <gos/arduino/test/tools/modbus/gam.h>

namespace gatt = ::gos::arduino::test::tools;

namespace ga = ::gos::atl;
namespace gatlm = ::gos::atl::modbus;

namespace gos {
namespace arduino {
namespace test {
namespace tools {
namespace modbus {

gam::gam(Stream& stream, const int& id, const size_t& size) :
  stream_(stream),
  handler_(id, size) {
}

void gam::begin(const int& baud) {
  gatlm::begin<Type>(stream_, handler_.parameter_, handler_.variable_, baud);
}

void gam::loop() {
  Type lr = gatlm::loop<Type>(
    stream_,
    handler_.parameter_,
    handler_,
    handler_.variable_,
    handler_.request_,
    handler_.response_);
}

Handler::Handler(const int& id, const size_t& size) :
  request_(static_cast<Type>(size)),
  response_(static_cast<Type>(size)) {
  parameter_.Control = 0;
  parameter_.Id = id;
}

MODBUS_TYPE_RESULT Handler::ReadCoils(
  const MODBUS_TYPE_FUNCTION& function,
  const Type& address,
  const Type& length) {
  std::cout << "GOS Modbus Function " << static_cast<int>(function)
    << ": Reading " << length
    << " coils from address " << address << std::endl;
  for (Type i = 0; i < length; ++i) {
    gatlm::provide::coil(variable_, request_, response_, i, false);
  }
  return MODBUS_STATUS_OK;
}

MODBUS_TYPE_RESULT Handler::ReadDiscreteInputs(
  const MODBUS_TYPE_FUNCTION& function,
  const Type& address,
  const Type& length) {
  std::cout << "GOS Modbus Function " << static_cast<int>(function)
    << ": Reading " << length
    << " discrete inputs from address " << address << std::endl;
  for (Type i = 0; i < length; ++i) {
    gatlm::provide::discrete(variable_, request_, response_, i, false);
  }
  return MODBUS_STATUS_OK;
}

MODBUS_TYPE_RESULT Handler::ReadHoldingRegisters(
  const MODBUS_TYPE_FUNCTION& function,
  const Type& address,
  const Type& length) {
  return MODBUS_STATUS_OK;
}

MODBUS_TYPE_RESULT Handler::ReadInputRegisters(
  const MODBUS_TYPE_FUNCTION& function,
  const Type& address,
  const Type& length) {
  return MODBUS_STATUS_OK;
}

MODBUS_TYPE_RESULT Handler::WriteCoils(
  const MODBUS_TYPE_FUNCTION& function,
  const Type& address,
  const Type& length) {
  return MODBUS_STATUS_OK;
}

MODBUS_TYPE_RESULT Handler::WriteHoldingRegisters(
  const MODBUS_TYPE_FUNCTION& function,
  const Type& address,
  const Type& length) {
  return MODBUS_STATUS_OK;
}

MODBUS_TYPE_RESULT Handler::ReadExceptionStatus(
  const MODBUS_TYPE_FUNCTION& function) {
  return MODBUS_STATUS_OK;
}


} // namespace modbus
} // namespace tools
} // namespace test
} // namespace arduino 
} // namespace gos
