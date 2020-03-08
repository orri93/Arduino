#include <Arduino.h>
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
namespace temperature {
namespace modbus {

void initialize() {
  parameter.Id = MODBUS_SLAVE_ID;
  parameter.Control = PIN_RS485_MODBUS_TE;
  gatl::modbus::begin<>(Serial, parameter, variable, MODBUS_BAUD);
}

Handler handler;

/* 0x01 Read Coils */
MODBUS_TYPE_RESULT gm::Handler::ReadCoils(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  MODBUS_TYPE_RESULT result = MODBUS_STATUS_OK;
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  for (MODBUS_TYPE_DEFAULT i = 0; i < length; ++i) {
    if (address + i < 8) {
      gatl::modbus::provide::coil<>(
        gm::variable,
        gm::buffer::request,
        gm::buffer::response,
        i,
        bitRead(gm::variables::coils, address + i));
    } else {
      result = MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
      break;
    }
  }
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return result;
}

/* 0x02 Read Discretes */
MODBUS_TYPE_RESULT gm::Handler::ReadDiscreteInputs(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  MODBUS_TYPE_RESULT result = MODBUS_STATUS_OK;
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  if (address == 0 && length == 1) {
    gatl::modbus::provide::discrete<MODBUS_TYPE_DEFAULT>(
      gm::variable,
      gm::buffer::request,
      gm::buffer::response,
      0,
      analogRead(PIN_BUTTON) < 512);
  } else {
    result = MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  }
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return result;
}

/* 0x03 Read Multiple Holding Registers */
MODBUS_TYPE_RESULT gm::Handler::ReadHoldingRegisters(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  uint8_t hb, lb;
  MODBUS_TYPE_DEFAULT reg;
  MODBUS_TYPE_RESULT result = MODBUS_STATUS_OK;
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  for (MODBUS_TYPE_DEFAULT i = 0; i < length; ++i) {
    if (address + i < 8) {
      lb = EEPROM.read(2 * (address + i));
      hb = EEPROM.read(1 + 2 * (address + i));
      reg = word(hb, lb);
      gatl::modbus::provide::registers<>(
        gm::variable,
        gm::buffer::request,
        gm::buffer::response,
        i,
        reg);
    } else {
      result = MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
      break;
    }
  }
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return result;
}

/* 0x04 Read Input Registers */
MODBUS_TYPE_RESULT gm::Handler::ReadInputRegisters(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  MODBUS_TYPE_RESULT result = MODBUS_STATUS_OK;
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  if (address == 0 && length == 1) {
    gatl::modbus::provide::registers<MODBUS_TYPE_DEFAULT>(
      gm::variable,
      gm::buffer::request,
      gm::buffer::response,
      0,
      analogRead(A2));
  } else {
    result = MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  }
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return result;
}

/* 0x05 Write Single Coil and 0x0f Write Multiple Coils */
MODBUS_TYPE_RESULT gm::Handler::WriteCoils(
  const MODBUS_TYPE_FUNCTION& function,
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  MODBUS_TYPE_RESULT result = MODBUS_STATUS_OK;
  digitalWrite(PIN_LED_MODBUS_WRITE, HIGH);
  for (MODBUS_TYPE_DEFAULT i = 0; i < length; ++i) {
    if (address + i < 8) {
      if (gatl::modbus::access::coil<>(gm::variable, gm::buffer::request, i)) {
        bitSet(gm::variables::coils, address + i);
      } else {
        bitClear(gm::variables::coils, address + i);
      }
    } else {
      result = MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
      break;
    }
  }
  digitalWrite(PIN_LED_MODBUS_WRITE, LOW);
  return result;
}

/* 0x06 Write Single and 0x10 Write Multiple Holding Registers */
MODBUS_TYPE_RESULT gm::Handler::WriteHoldingRegisters(
  const MODBUS_TYPE_FUNCTION& function,
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  uint8_t hb, lb;
  MODBUS_TYPE_DEFAULT reg;
  MODBUS_TYPE_RESULT result = MODBUS_STATUS_OK;
  digitalWrite(PIN_LED_MODBUS_WRITE, HIGH);
  for (MODBUS_TYPE_DEFAULT i = 0; i < length; ++i) {
    if (address + i < 8) {
      reg = gatl::modbus::access::registers<>(
        gm::variable,
        gm::buffer::request,
        i);
      lb = lowByte(reg);
      hb = highByte(reg);
      EEPROM.update(2 * (address + i), lb);
      EEPROM.update(1 + 2 * (address + i), hb);
      if (address + i == 0) {
        gm::variables::led::blue::value = lb;
      }
    } else {
      result = MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
      break;
    }
  }
  digitalWrite(PIN_LED_MODBUS_WRITE, LOW);
  return result;
}

MODBUS_TYPE_RESULT gm::Handler::ReadExceptionStatus() {
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return MODBUS_STATUS_OK;
}

} // namespace modbus
} // namespace temperature
} // namespace gos
