#include <EEPROM.h>

#include "macro.h"
#include "variable.h"

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

  gatl::modbus::callback::set::read::coils(
    &::gos::modbus::callback::read::coils);

  gatl::modbus::callback::set::write::coils(
    &::gos::modbus::callback::write::coils);
}

namespace buffer {
Holder request(MODBUS_BUFFER_SIZE);
Holder response(MODBUS_BUFFER_SIZE);
} // namespace buffer

Parameter parameter;
Variable variable;

#ifdef MODBUS_HANDLER_INTERFACE
Handler handler;

/* 0x01 Read Coils */
MODBUS_TYPE_RESULT gm::Handler::ReadCoils(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
#ifdef NOT_YET
  for (MODBUS_TYPE_DEFAULT i = 0; i < length; ++i) {
    if (address + i < 8) {
      gatl::modbus::provide::coil<>(
        gm::variable,
        gm::buffer::request,
        gm::buffer::response,
        i,
        bitRead(gm::variables::coils, address + i));
    } else {
      return MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
    }
  }
#endif
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return MODBUS_STATUS_OK;
}

#define TEXT_DISCR_0_IS_0 "RD:0=0"
#define TEXT_DISCR_0_IS_1 "RD:0=1"
#define TEXT_DISCR_0_IS_E "RD:0=E"
#define TEXT_DISCR_1_IS_0 "RD:1=0"
#define TEXT_DISCR_1_IS_1 "RD:1=1"
#define TEXT_DISCR_1_IS_E "RD:1=E"

/* 0x02 Read Discretes */
MODBUS_TYPE_RESULT gm::Handler::ReadDiscreteInputs(
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
      0x0001,
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
#ifdef NOT_YET
  for (MODBUS_TYPE_DEFAULT i = 0; i < length; ++i) {
    if (address + i < 256) {
      uint8_t lb = EEPROM.read(2 * (address + i));
      uint8_t hb = EEPROM.read(2 * (address + i) + 1);
      gatl::modbus::provide::registers<>(
        gm::variable,
        gm::buffer::request,
        gm::buffer::response,
        i,
        word(hb, lb));
    } else {
      return MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
    }
  }
#endif
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return MODBUS_STATUS_OK;
}

/* 0x04 Read Input Registers */
MODBUS_TYPE_RESULT gm::Handler::ReadInputRegisters(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return MODBUS_STATUS_OK;
}

/* 0x05 Write Single Coil and 0x0f Write Multiple Coils */
MODBUS_TYPE_RESULT gm::Handler::WriteCoils(
  const MODBUS_TYPE_FUNCTION& function,
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  digitalWrite(PIN_LED_MODBUS_WRITE, HIGH);
#ifdef NOT_YET
  for (MODBUS_TYPE_DEFAULT i = 0; i < length; ++i) {
    if (address + i < 8) {
      if (gatl::modbus::access::coil<>(gm::variable, gm::buffer::request, i)) {
        bitSet(gm::variables::coils, address + i);
      } else {
        bitClear(gm::variables::coils, address + i);
      }
    } else {
      return MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
    }
  }
#endif
  digitalWrite(PIN_LED_MODBUS_WRITE, LOW);
  return MODBUS_STATUS_OK;
}

/* 0x06 Write Single and 0x10 Write Multiple Holding Registers */
MODBUS_TYPE_RESULT gm::Handler::WriteHoldingRegisters(
  const MODBUS_TYPE_FUNCTION& function,
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  digitalWrite(PIN_LED_MODBUS_WRITE, HIGH);
#ifdef NOT_YET
  for (MODBUS_TYPE_DEFAULT i = 0; i < length; ++i) {
    if (address + i < 256) {
      MODBUS_TYPE_DEFAULT r = gatl::modbus::access::registers<>(
        gm::variable,
        gm::buffer::request,
        i);
      EEPROM.update(2 * (address + i), lowByte(r));
      EEPROM.update(2 * (address + i) + 1, highByte(r));
      if (address + i == 0) {
        gm::variables::led::blue::a = lowByte(r);
        gm::variables::led::blue::b = highByte(r);
      }
    } else {
      return MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
    }
  }
#endif
  digitalWrite(PIN_LED_MODBUS_WRITE, LOW);
  return MODBUS_STATUS_OK;
}

MODBUS_TYPE_RESULT gm::Handler::ReadExceptionStatus() {
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return MODBUS_STATUS_OK;
}
#else
namespace callback {
namespace read {
MODBUS_TYPE_RESULT coils(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return MODBUS_STATUS_OK;
}
namespace discrete {
static MODBUS_TYPE_RESULT inputs(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return MODBUS_STATUS_OK;
}
}
namespace holding {
static MODBUS_TYPE_RESULT registers(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return MODBUS_STATUS_OK;
}
}
namespace input {
static MODBUS_TYPE_RESULT registers(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return MODBUS_STATUS_OK;
}
}
namespace exception {
static MODBUS_TYPE_RESULT status() {
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return MODBUS_STATUS_OK;
}
}
}
namespace write {
static MODBUS_TYPE_RESULT coils(
  const MODBUS_TYPE_FUNCTION& function,
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  digitalWrite(PIN_LED_MODBUS_WRITE, HIGH);
  digitalWrite(PIN_LED_MODBUS_WRITE, LOW);
  return MODBUS_STATUS_OK;
}
namespace holding {
static MODBUS_TYPE_RESULT registers(
  const MODBUS_TYPE_FUNCTION& function,
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  digitalWrite(PIN_LED_MODBUS_WRITE, HIGH);
  digitalWrite(PIN_LED_MODBUS_WRITE, LOW);
  return MODBUS_STATUS_OK;
}
}
}
}
#endif

} // namespace modbus
} // namespace gos
