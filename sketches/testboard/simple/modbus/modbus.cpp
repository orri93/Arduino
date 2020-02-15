#include "macro.h"

#ifndef NO_DISPLAY
#include "format.h"
#include "display.h"
#endif

#include "modbus.h"

namespace gatl = ::gos::atl;
namespace gatlu = ::gos::atl::utility;

namespace gm = ::gos::modbus;

#ifndef NO_DISPLAY
namespace gmd = ::gos::modbus::display;
#endif

namespace gos {
namespace modbus {

void initialize() {
  parameter.Id = 1;
  parameter.Control = PIN_RS485_MODBUS_TE;
}

namespace buffer {
Holder request(MODBUS_BUFFER_SIZE);
Holder response(MODBUS_BUFFER_SIZE);
} // namespace buffer

Parameter parameter;
Variable variable;

/* 0x01 Read Coils */
MODBUS_TYPE_RESULT gm::Handler::ReadCoils(
  const MODBUS_TYPE_DEFAULT& start,
  const MODBUS_TYPE_DEFAULT& length) {
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
#ifdef GOS_MODBUS_DO_NOTHING
  return MODBUS_STATUS_OK;
#else
  ::gos::atl::modbus::binding::result result =
    gatl::modbus::binding::coil::access<>(
    gm::modbus::binding::coils,
    gm::modbus::variable,
    gm::modbus::buffer::request,
    gm::modbus::buffer::response,
    start,
    length);
  switch (result) {
  case ::gos::atl::modbus::binding::result::excluded:
    return MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  case ::gos::atl::modbus::binding::result::failure:
    return MODBUS_STATUS_SLAVE_DEVICE_FAILURE;
  case ::gos::atl::modbus::binding::result::included:
    return MODBUS_STATUS_OK;
  default:
    return MODBUS_STATUS_SLAVE_DEVICE_FAILURE;
  }
#endif
}

#define TEXT_DISCR_0_IS_0 "RD:0=0"
#define TEXT_DISCR_0_IS_1 "RD:0=1"
#define TEXT_DISCR_0_IS_E "RD:0=E"
#define TEXT_DISCR_1_IS_0 "RD:1=0"
#define TEXT_DISCR_1_IS_1 "RD:1=1"
#define TEXT_DISCR_1_IS_E "RD:1=E"

/* 0x02 Read Discretes */
MODBUS_TYPE_RESULT gm::Handler::ReadDiscretes(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  bool bs;
  uint8_t result = MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  if(gatl::utility::range::ismemberof<uint16_t>(0x0000, address, length)) {
    bs = digitalRead(PIN_BUTTON_A) == LOW;
    result = gatl::modbus::provide::discrete<uint16_t>(
      gm::variable,
      gm::buffer::request,
      gm::buffer::response,
      0x0000,
      bs);
    if (result == MODBUS_STATUS_OK) {
      gm::display::update::first::line(
        bs ? TEXT_DISCR_0_IS_1 : TEXT_DISCR_0_IS_0);
    } else {
      gm::display::update::first::line(TEXT_DISCR_0_IS_E);
      digitalWrite(PIN_LED_MODBUS_READ, LOW);
      return result;
    }
  }
  if (gatl::utility::range::ismemberof<uint16_t>(0x0001, address, length)) {
    bs = digitalRead(PIN_BUTTON_B) == LOW;
    result = gatl::modbus::provide::discrete<uint16_t>(
      gm::variable,
      gm::buffer::request,
      gm::buffer::response,
      0x0000,
      bs);
    if (result == MODBUS_STATUS_OK) {
      gm::display::update::second::line(
        bs ? TEXT_DISCR_1_IS_1 : TEXT_DISCR_1_IS_0);
    } else {
      gm::display::update::second::line(TEXT_DISCR_1_IS_E);
      digitalWrite(PIN_LED_MODBUS_READ, LOW);
      return result;
    }
  }
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return result;
}

/* 0x03 Read Multiple Holding Registers */
MODBUS_TYPE_RESULT gm::Handler::ReadHoldingRegisters(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
#ifdef GOS_MODBUS_DO_NOTHING
  return MODBUS_STATUS_OK;

#else
  ::gos::atl::modbus::binding::result ro =
    gatl::modbus::binding::registers::access<>(
      gm::binding::barray::output,
      gm::modbus::variable,
      gm::modbus::buffer::request,
      gm::modbus::buffer::response,
      start,
      length);
  ::gos::atl::modbus::binding::result rr =
    gatl::modbus::binding::two::access<>(
      gm::binding::barray::real,
      gm::modbus::variable,
      gm::modbus::buffer::request,
      gm::modbus::buffer::response,
      start,
      length);
  if (ro == ::gos::atl::modbus::binding::result::included ||
    rr == ::gos::atl::modbus::binding::result::included) {
    return MODBUS_STATUS_OK;
  } else if (ro == ::gos::atl::modbus::binding::result::failure ||
    rr == ::gos::atl::modbus::binding::result::failure) {
    return MODBUS_STATUS_SLAVE_DEVICE_FAILURE;
  } else if(ro == ::gos::atl::modbus::binding::result::excluded &&
    rr == ::gos::atl::modbus::binding::result::excluded) {
    return MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  } else {
    return MODBUS_STATUS_SLAVE_DEVICE_FAILURE;
  }
#endif
}
/* 0x04 Read Input Registers */
MODBUS_TYPE_RESULT gm::Handler::ReadInputRegisters(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
#ifdef GOS_MODBUS_DO_NOTHING
  return MODBUS_STATUS_OK;
#else
  ::gos::atl::modbus::binding::result ro =
    gatl::modbus::binding::registers::access<gm::type::Output>(
      gm::modbus::binding::input::output,
      gm::modbus::variable,
      gm::modbus::buffer::request,
      gm::modbus::buffer::response,
      start,
      length);
  ::gos::atl::modbus::binding::result rr =
    gatl::modbus::binding::two::access<gm::type::Real>(
      gm::modbus::binding::input::sensor,
      gm::modbus::variable,
      gm::modbus::buffer::request,
      gm::modbus::buffer::response,
      start,
      length);
  if (ro == ::gos::atl::modbus::binding::result::included ||
    rr == ::gos::atl::modbus::binding::result::included) {
    return MODBUS_STATUS_OK;
  } else if (ro == ::gos::atl::modbus::binding::result::failure ||
    rr == ::gos::atl::modbus::binding::result::failure) {
    return MODBUS_STATUS_SLAVE_DEVICE_FAILURE;
  } else if (ro == ::gos::atl::modbus::binding::result::excluded &&
    rr == ::gos::atl::modbus::binding::result::excluded) {
    return MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  } else {
    return MODBUS_STATUS_SLAVE_DEVICE_FAILURE;
  }
#endif
}

/* 0x05 Write Single Coil and 0x0f Write Multiple Coils */
MODBUS_TYPE_RESULT gm::Handler::WriteCoils(
  const MODBUS_TYPE_FUNCTION& function,
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  digitalWrite(PIN_LED_MODBUS_WRITE, HIGH);
  digitalWrite(PIN_LED_MODBUS_WRITE, LOW);
#ifdef GOS_MODBUS_DO_NOTHING
  return MODBUS_STATUS_OK;
#else
  ::gos::modbus::mode::gotom::state(::gos::modbus::mode::status::coil);
  uint16_t address, first, last;
  uint8_t index = 0;
  ::gos::atl::modbus::binding::result result =
    gatl::modbus::binding::coil::assign<>(
    gm::modbus::binding::coils,
    gm::modbus::variable,
    gm::modbus::buffer::request,
    gm::modbus::buffer::response,
    start,
    length,
    address,
    first,
    last,
    index);
  switch (result) {
  case ::gos::atl::modbus::binding::result::excluded:
    return MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  case ::gos::atl::modbus::binding::result::failure:
    return MODBUS_STATUS_SLAVE_DEVICE_FAILURE;
  case ::gos::atl::modbus::binding::result::included:
    return MODBUS_STATUS_OK;
  default:
    return MODBUS_STATUS_SLAVE_DEVICE_FAILURE;
  }
#endif
}

/* 0x06 Write Single and 0x10 Write Multiple Holding Registers */
MODBUS_TYPE_RESULT gm::Handler::WriteHoldingRegisters(
  const MODBUS_TYPE_FUNCTION& function,
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  digitalWrite(PIN_LED_MODBUS_WRITE, HIGH);
  digitalWrite(PIN_LED_MODBUS_WRITE, LOW);
#ifdef GOS_MODBUS_DO_NOTHING
  return MODBUS_STATUS_OK;
#else
  uint16_t address, first, last;
  uint8_t index = 0;
  ::gos::atl::modbus::binding::result ro =
    gatl::modbus::binding::registers::assign<type::Output>(
      gm::binding::barray::output,
      gm::modbus::variable,
      gm::modbus::buffer::request,
      gm::modbus::buffer::response,
      start,
      length,
      address,
      first,
      last,
      index);
  ::gos::atl::modbus::binding::result rr =
    gatl::modbus::binding::two::assign<type::Real>(
      gm::binding::barray::real,
      gm::modbus::variable,
      gm::modbus::buffer::request,
      gm::modbus::buffer::response,
      start,
      length,
      address,
      first,
      last,
      index);
  if (ro == ::gos::atl::modbus::binding::result::included ||
    rr == ::gos::atl::modbus::binding::result::included) {
    return MODBUS_STATUS_OK;
  } else if (ro == ::gos::atl::modbus::binding::result::failure ||
    rr == ::gos::atl::modbus::binding::result::failure) {
    return MODBUS_STATUS_SLAVE_DEVICE_FAILURE;
  } else if (ro == ::gos::atl::modbus::binding::result::excluded &&
    rr == ::gos::atl::modbus::binding::result::excluded) {
    return MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  } else {
    return MODBUS_STATUS_SLAVE_DEVICE_FAILURE;
  }

#endif
  return MODBUS_STATUS_OK;
}
MODBUS_TYPE_RESULT gm::Handler::ReadExceptionStatus() {
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return MODBUS_STATUS_OK;
}

} // namespace modbus
} // namespace gos
