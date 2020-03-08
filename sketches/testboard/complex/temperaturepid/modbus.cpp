#include <Arduino.h>
#include <EEPROM.h>

#include <gatlstring.h>

#include "macro.h"
#include "variable.h"
#include "display.h"
#include "binding.h"
#include "format.h"
#include "modbus.h"
#include "text.h"
#include "pid.h"

namespace gatl = ::gos::atl;
namespace gatlu = ::gos::atl::utility;
namespace gatlmb = ::gos::atl::modbus::binding;

namespace gt = ::gos::temperature;
namespace gtvm = ::gos::temperature::variables::modbus;
namespace gtvt = ::gos::temperature::variables::temporary;
namespace gtfdb = ::gos::temperature::format::display::buffer;

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
MODBUS_TYPE_RESULT gt::Handler::ReadCoils(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  MODBUS_TYPE_RESULT result = MODBUS_STATUS_OK;
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  for (MODBUS_TYPE_DEFAULT i = 0; i < length; ++i) {
    if (address + i < 1) {
      gatl::modbus::provide::coil<>(
        gt::variable,
        gt::buffer::request,
        gt::buffer::response,
        i,
        bitRead(gt::variables::modbus::coils, address + i));
    } else {
      result = MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
      break;
    }
  }
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return result;
}

/* 0x02 Read Discretes */
MODBUS_TYPE_RESULT gt::Handler::ReadDiscreteInputs(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  return MODBUS_STATUS_ILLEGAL_FUNCTION;
}

/* 0x03 Read Multiple Holding Registers */
MODBUS_TYPE_RESULT gt::Handler::ReadHoldingRegisters(
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
        gt::variable,
        gt::buffer::request,
        gt::buffer::response,
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
MODBUS_TYPE_RESULT gt::Handler::ReadInputRegisters(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  gatlmb::result uints, reals;
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  uints = gatl::modbus::binding::registers::access(
    gt::binding::modbus::input::registers::uints,
    gt::modbus::variable,
    gt::modbus::buffer::request,
    gt::modbus::buffer::response,
    address,
    length);
  reals = gatl::modbus::binding::two::access(
    gt::binding::modbus::input::registers::real,
    gt::modbus::variable,
    gt::modbus::buffer::request,
    gt::modbus::buffer::response,
    address,
    length);
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  if (uints == gatlmb::result::failure || reals == gatlmb::result::failure) {
    return MODBUS_STATUS_SLAVE_DEVICE_FAILURE;
  }
  if (uints == gatlmb::result::included || reals == gatlmb::result::included) {
    return MODBUS_STATUS_OK;
  } else {
    return MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  }
}

/* 0x05 Write Single Coil and 0x0f Write Multiple Coils */
MODBUS_TYPE_RESULT gt::Handler::WriteCoils(
  const MODBUS_TYPE_FUNCTION& function,
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  uint8_t last = gt::variables::modbus::coils;
  MODBUS_TYPE_RESULT result = MODBUS_STATUS_OK;
  digitalWrite(PIN_LED_MODBUS_WRITE, HIGH);
  for (MODBUS_TYPE_DEFAULT i = 0; i < length; ++i) {
    if (address + i < 1) {
      if (gatl::modbus::access::coil<>(gt::variable, gt::buffer::request, i)) {
        bitSet(gt::variables::modbus::coils, address + i);
      } else {
        bitClear(gt::variables::modbus::coils, address + i);
      }
      if (address + i == GOS_TCV_COIL_BIT_PONE) {
        gtvt::boolean = bitRead(gtvm::coils, GOS_TCV_COIL_BIT_PONE);
        if (gtvt::boolean != gt::pid::parameter.PonE) {
          gt::pid::parameter.PonE = gtvt::boolean;
          if (gt::pid::parameter.PonE) {
            GATL_TEXT_MEMCPY(gtfdb::first, GOS_TCT_P_ON_E);
          } else {
            GATL_TEXT_MEMCPY(gtfdb::first, GOS_TCT_P_ON_E_NOT);
          }
          GATL_TEXT_MEMCPY(gtfdb::second, GOS_TCT);
          gt::display::two.display(gtfdb::first, gtfdb::second);
        }
      } else if (address + i == GOS_TCV_COIL_BIT_TUNE_TIME_MASTER) {
        gtvt::boolean = bitRead(gtvm::coils, GOS_TCV_COIL_BIT_TUNE_TIME_MASTER);
        if (bitRead(last, GOS_TCV_COIL_BIT_TUNE_TIME_MASTER) != gtvt::boolean) {
          if (gt::pid::parameter.PonE) {
            GATL_TEXT_MEMCPY(gtfdb::first, GOS_TCT_TUNE_T);
          } else {
            GATL_TEXT_MEMCPY(gtfdb::first, GOS_TCT_TUNE_K);
          }
          GATL_TEXT_MEMCPY(gtfdb::second, GOS_TCT);
          gt::display::two.display(gtfdb::first, gtfdb::second);
        }
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
MODBUS_TYPE_RESULT gt::Handler::WriteHoldingRegisters(
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
        gt::variable,
        gt::buffer::request,
        i);
      lb = lowByte(reg);
      hb = highByte(reg);
      EEPROM.update(2 * (address + i), lb);
      EEPROM.update(1 + 2 * (address + i), hb);
      if (address + i == 0) {
        gt::variables::led::blue::value = lb;
      }
    } else {
      result = MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
      break;
    }
  }
  digitalWrite(PIN_LED_MODBUS_WRITE, LOW);
  return result;
}

MODBUS_TYPE_RESULT gt::Handler::ReadExceptionStatus() {
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return MODBUS_STATUS_OK;
}

} // namespace modbus
} // namespace temperature
} // namespace gos
