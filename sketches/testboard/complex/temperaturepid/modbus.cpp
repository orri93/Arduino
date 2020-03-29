#include <Arduino.h>
#include <EEPROM.h>

#include <gatlstring.h>
#include <gatlutility.h>
#include <gatlformat.h>

#include "macro.h"
#include "variable.h"
#include "display.h"
#include "format.h"
#include "modbus.h"
#include "eeprom.h"
#include "sensor.h"
#include "value.h"
#include "text.h"
#include "pid.h"

#define GOS_TC_IRA_OUTPUT    0x0000
#define GOS_TC_IRA_TEMP      0x0001

#define GOS_TC_HRA_INTERVAL  0x0000
#define GOS_TC_HRA_MANUAL    0x0001
#define GOS_TC_HRA_SETPOINT  0x0002
#define GOS_TC_HRA_KP        0x0004
#define GOS_TC_HRA_I         0x0006
#define GOS_TC_HRA_D         0x0008
#define GOS_TC_HRA_MIN_SENS  0x000A
#define GOS_TC_HRA_MAX_SENS  0x000C

namespace gatl = ::gos::atl;
namespace gatlu = ::gos::atl::utility;
namespace gatlur = ::gos::atl::utility::range;
namespace gatluri = ::gos::atl::utility::range::inclusive;
namespace gatlun = ::gos::atl::utility::number;
namespace gatlmb = ::gos::atl::modbus::binding;
namespace gtp = ::gos::temperature::pid;

namespace gt = ::gos::temperature;
namespace gtm = ::gos::temperature::modbus;
namespace gtv = ::gos::temperature::variables;
namespace gtvi = ::gos::temperature::variables::timing;
namespace gtvm = ::gos::temperature::variables::modbus;
namespace gtvc = ::gos::temperature::variables::controller;
namespace gtvt = ::gos::temperature::variables::temporary;
namespace gtfdb = ::gos::temperature::format::display::buffer;
namespace gtf = ::gos::temperature::format;

namespace gos {
namespace temperature {
namespace modbus {

Handler handler;

namespace buffer {
Holder request(MODBUS_BUFFER_SIZE);
Holder response(MODBUS_BUFFER_SIZE);
} // namespace buffer
Parameter parameter;
Variable variable;

void initialize() {
  parameter.Id = MODBUS_SLAVE_ID;
  parameter.Control = PIN_RS485_MODBUS_TE;
  gatl::modbus::begin<>(Serial, parameter, variable, MODBUS_BAUD);
}

/* 0x01 Read Coils */
MODBUS_TYPE_RESULT gtm::Handler::ReadCoils(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  MODBUS_TYPE_RESULT result = MODBUS_STATUS_OK;
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  for (MODBUS_TYPE_DEFAULT i = 0; i < length; ++i) {
    if (address + i < 1) {
      gatl::modbus::provide::coil<>(
        gtm::variable,
        gtm::buffer::request,
        gtm::buffer::response,
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
MODBUS_TYPE_RESULT gtm::Handler::ReadDiscreteInputs(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  return MODBUS_STATUS_ILLEGAL_FUNCTION;
}

/* 0x03 Read Multiple Holding Registers */
MODBUS_TYPE_RESULT gtm::Handler::ReadHoldingRegisters(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  MODBUS_TYPE_BUFFER* location;
  MODBUS_TYPE_RESULT result = MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  if (location = gatl::modbus::provide::buffer::location<MODBUS_TYPE_DEFAULT>(
    gtm::variable,
    gtm::buffer::request,
    gtm::buffer::response,
    GOS_TC_HRA_INTERVAL,
    1,
    address,
    length)) {
    MODBUS_WRITE_UINT16_AT0(location, gt::variables::timing::interval);
    result = MODBUS_STATUS_OK;
  }
  if (location = gatl::modbus::provide::buffer::location<MODBUS_TYPE_DEFAULT>(
    gtm::variable,
    gtm::buffer::request,
    gtm::buffer::response,
    GOS_TC_HRA_MANUAL,
    1,
    address,
    length)) {
    MODBUS_WRITE_UINT16_AT0(location, gt::variables::controller::manual);
    result = MODBUS_STATUS_OK;
  }
  if (location = gatl::modbus::provide::buffer::location<MODBUS_TYPE_DEFAULT>(
    gtm::variable,
    gtm::buffer::request,
    gtm::buffer::response,
    GOS_TC_HRA_SETPOINT,
    2,
    address,
    length)) {
    gt::variables::temporary::integer =
      gatl::utility::number::part::first<uint16_t, type::Real>(
        gtp::parameter.Setpoint);
    MODBUS_WRITE_UINT16_AT0(location, gt::variables::temporary::integer);
    gt::variables::temporary::integer =
      gatl::utility::number::part::second<uint16_t, type::Real>(
        gtp::parameter.Setpoint);
    MODBUS_WRITE_UINT16_AT0(location + 2, gt::variables::temporary::integer);
    result = MODBUS_STATUS_OK;
  }
  if (location = gatl::modbus::provide::buffer::location<MODBUS_TYPE_DEFAULT>(
    gtm::variable,
    gtm::buffer::request,
    gtm::buffer::response,
    GOS_TC_HRA_KP,
    2,
    address,
    length)) {
    gt::variables::temporary::integer =
      gatl::utility::number::part::first<uint16_t, type::Real>(
        gtp::parameter.Kp);
    MODBUS_WRITE_UINT16_AT0(location, gt::variables::temporary::integer);
    gt::variables::temporary::integer =
      gatl::utility::number::part::second<uint16_t, type::Real>(
        gtp::parameter.Kp);
    MODBUS_WRITE_UINT16_AT0(location + 2, gt::variables::temporary::integer);
    result = MODBUS_STATUS_OK;
  }
  if (location = gatl::modbus::provide::buffer::location<MODBUS_TYPE_DEFAULT>(
    gtm::variable,
    gtm::buffer::request,
    gtm::buffer::response,
    GOS_TC_HRA_I,
    2,
    address,
    length)) {
    if (bitRead(gtv::modbus::coils, GOS_TCV_COIL_BIT_TUNE_TIME_MASTER)) {
      gt::variables::temporary::integer =
        gatl::utility::number::part::first<uint16_t, type::Real>(
          gtp::tune::k.Ki);
      MODBUS_WRITE_UINT16_AT0(location, gt::variables::temporary::integer);
      gt::variables::temporary::integer =
        gatl::utility::number::part::second<uint16_t, type::Real>(
          gtp::tune::k.Ki);
      MODBUS_WRITE_UINT16_AT0(location + 2, gt::variables::temporary::integer);
    } else {
      gt::variables::temporary::integer =
        gatl::utility::number::part::first<uint16_t, type::Real>(
          gtp::tune::t.Ti);
      MODBUS_WRITE_UINT16_AT0(location, gt::variables::temporary::integer);
      gt::variables::temporary::integer =
        gatl::utility::number::part::second<uint16_t, type::Real>(
          gtp::tune::t.Ti);
      MODBUS_WRITE_UINT16_AT0(location + 2, gt::variables::temporary::integer);
    }
    result = MODBUS_STATUS_OK;
  }
  if (location = gatl::modbus::provide::buffer::location<MODBUS_TYPE_DEFAULT>(
    gtm::variable,
    gtm::buffer::request,
    gtm::buffer::response,
    GOS_TC_HRA_D,
    2,
    address,
    length)) {
    if (bitRead(gtv::modbus::coils, GOS_TCV_COIL_BIT_TUNE_TIME_MASTER)) {
      gt::variables::temporary::integer =
        gatl::utility::number::part::first<uint16_t, type::Real>(
          gtp::tune::k.Kd);
      MODBUS_WRITE_UINT16_AT0(location, gt::variables::temporary::integer);
      gt::variables::temporary::integer =
        gatl::utility::number::part::second<uint16_t, type::Real>(
          gtp::tune::k.Kd);
      MODBUS_WRITE_UINT16_AT0(location + 2, gt::variables::temporary::integer);
    } else {
      gt::variables::temporary::integer =
        gatl::utility::number::part::first<uint16_t, type::Real>(
          gtp::tune::t.Td);
      MODBUS_WRITE_UINT16_AT0(location, gt::variables::temporary::integer);
      gt::variables::temporary::integer =
        gatl::utility::number::part::second<uint16_t, type::Real>(
          gtp::tune::t.Td);
      MODBUS_WRITE_UINT16_AT0(location + 2, gt::variables::temporary::integer);
    }
    result = MODBUS_STATUS_OK;
  }
  if (location = gatl::modbus::provide::buffer::location<MODBUS_TYPE_DEFAULT>(
    gtm::variable,
    gtm::buffer::request,
    gtm::buffer::response,
    GOS_TC_HRA_MIN_SENS,
    2,
    address,
    length)) {
    gt::variables::temporary::integer =
      gatl::utility::number::part::first<uint16_t, double>(
        gt::sensor::max6675sensor.Range.lowest);
    MODBUS_WRITE_UINT16_AT0(location, gt::variables::temporary::integer);
    gt::variables::temporary::integer =
      gatl::utility::number::part::second<uint16_t, double>(
        gt::sensor::max6675sensor.Range.lowest);
    MODBUS_WRITE_UINT16_AT0(location + 2, gt::variables::temporary::integer);
    result = MODBUS_STATUS_OK;
  }
  if (location = gatl::modbus::provide::buffer::location<MODBUS_TYPE_DEFAULT>(
    gtm::variable,
    gtm::buffer::request,
    gtm::buffer::response,
    GOS_TC_HRA_MAX_SENS,
    2,
    address,
    length)) {
    gt::variables::temporary::integer =
      gatl::utility::number::part::first<uint16_t, double>(
        gt::sensor::max6675sensor.Range.highest);
    MODBUS_WRITE_UINT16_AT0(location, gt::variables::temporary::integer);
    gt::variables::temporary::integer =
      gatl::utility::number::part::second<uint16_t, double>(
        gt::sensor::max6675sensor.Range.highest);
    MODBUS_WRITE_UINT16_AT0(location + 2, gt::variables::temporary::integer);
    result = MODBUS_STATUS_OK;
  }
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return result;
}

/* 0x04 Read Input Registers */
MODBUS_TYPE_RESULT gtm::Handler::ReadInputRegisters(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  MODBUS_TYPE_BUFFER* location;
  MODBUS_TYPE_RESULT result = MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  if(location = gatl::modbus::provide::buffer::location<MODBUS_TYPE_DEFAULT>(
    gtm::variable,
    gtm::buffer::request,
    gtm::buffer::response,
    GOS_TC_IRA_OUTPUT,
    1,
    address,
    length)) {
    MODBUS_WRITE_UINT16_AT0(location, gt::variables::output);
    result = MODBUS_STATUS_OK;
  }
  if (location = gatl::modbus::provide::buffer::location<MODBUS_TYPE_DEFAULT>(
    gtm::variable,
    gtm::buffer::request,
    gtm::buffer::response,
    GOS_TC_IRA_TEMP,
    2,
    address,
    length)) {
    gt::variables::temporary::integer =
      gatl::utility::number::part::first<uint16_t, type::Real>(
        gt::variables::temperature);
    MODBUS_WRITE_UINT16_AT0(location, gt::variables::temporary::integer);
    gt::variables::temporary::integer =
      gatl::utility::number::part::second<uint16_t, type::Real>(
        gt::variables::temperature);
    MODBUS_WRITE_UINT16_AT0(location + 2, gt::variables::temporary::integer);
    result = MODBUS_STATUS_OK;
  }
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return result;
}

/* 0x05 Write Single Coil and 0x0f Write Multiple Coils */
MODBUS_TYPE_RESULT gtm::Handler::WriteCoils(
  const MODBUS_TYPE_FUNCTION& function,
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  bool displayed = false;
  uint8_t last = gt::variables::modbus::coils;
  MODBUS_TYPE_RESULT result = MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  digitalWrite(PIN_LED_MODBUS_WRITE, HIGH);
  for (MODBUS_TYPE_DEFAULT i = 0; i < length; ++i) {
    if (address + i < 1) {
      if (gatl::modbus::access::coil<>(gtm::variable, gtm::buffer::request, i)){
        bitSet(gt::variables::modbus::coils, address + i);
      } else {
        bitClear(gt::variables::modbus::coils, address + i);
      }
      if (address + i == GOS_TCV_COIL_BIT_PONE) {
        gtvt::boolean = bitRead(gtvm::coils, GOS_TCV_COIL_BIT_PONE);
        if (gtvt::boolean != gt::pid::parameter.PonE) {
          gt::pid::parameter.PonE = gtvt::boolean;
          if (!displayed) {
            if (gt::pid::parameter.PonE) {
              GATL_TEXT_MEMCPY(gtfdb::second.Buffer, GOS_TCT_P_ON_E);
            } else {
              GATL_TEXT_MEMCPY(gtfdb::second.Buffer, GOS_TCT_P_ON_E_NOT);
            }
            displayed = true;
          }
        }
        result = MODBUS_STATUS_OK;
      } else if (address + i == GOS_TCV_COIL_BIT_TUNE_TIME_MASTER) {
        gtvt::boolean = bitRead(gtvm::coils, GOS_TCV_COIL_BIT_TUNE_TIME_MASTER);
        if (bitRead(last, GOS_TCV_COIL_BIT_TUNE_TIME_MASTER) != gtvt::boolean) {
          if (!displayed) {
            if (gtvt::boolean) {
              GATL_TEXT_MEMCPY(gtfdb::second.Buffer, GOS_TCT_TUNE_T);
            } else {
              GATL_TEXT_MEMCPY(gtfdb::second.Buffer, GOS_TCT_TUNE_K);
            }
            displayed = true;
          }
        }
        result = MODBUS_STATUS_OK;
      }
    }
  }
  if (displayed) {
    gt::display::two.display(gtfdb::first, gtfdb::second);
  }
  digitalWrite(PIN_LED_MODBUS_WRITE, LOW);
  return result;
}

/* 0x06 Write Single and 0x10 Write Multiple Holding Registers */
MODBUS_TYPE_RESULT gtm::Handler::WriteHoldingRegisters(
  const MODBUS_TYPE_FUNCTION& function,
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  bool displayed = false;
  bool calculatetime = false;
  MODBUS_TYPE_BUFFER* location;
  MODBUS_TYPE_RESULT result = MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  digitalWrite(PIN_LED_MODBUS_WRITE, HIGH);
  if (location = gatl::modbus::access::buffer::location<MODBUS_TYPE_DEFAULT>(
    gtm::variable,
    gtm::buffer::request,
    GOS_TC_HRA_MANUAL,
    1,
    address,
    length)) {
    gt::variables::temporary::integer = MODBUS_READ_UINT16_AT0(location);
    if (gt::variables::temporary::integer != gt::variables::controller::manual) {
      if (gt::variables::temporary::integer <= gt::value::MaxManual) {
        gt::variables::controller::manual = gt::variables::temporary::integer;
        EEPROM.put<type::Unsigned>(GOS_TC_EEPROM_INDEX_MANUAL, gtvc::manual);
        if (!displayed) {
          gatl::format::integer<type::Unsigned, uint8_t>(
            gtfdb::second,
            gt::variables::controller::manual,
            &gt::format::display::buffer::text::manual);
          gt::variables::status = gt::type::Status::manual;
          displayed = true;
        }
      }
    }
    if (gt::variables::temporary::integer <= gt::value::MaxManual) {
      result = MODBUS_STATUS_OK;
    } else {
      result = MODBUS_STATUS_ILLEGAL_DATA_VALUE;
      goto gos_modbus_handler_write_coils_finaly;
    }
  }
  if (location = gatl::modbus::access::buffer::location<MODBUS_TYPE_DEFAULT>(
    gtm::variable,
    gtm::buffer::request,
    GOS_TC_HRA_SETPOINT,
    2,
    address,
    length)) {
    gtvt::real = gatlun::part::combine<uint16_t, type::Real>(
      MODBUS_READ_UINT16_AT0(location),
      MODBUS_READ_UINT16_AT0(location + 2));
    if (gtvt::real != gtp::parameter.Setpoint) {
      gtp::parameter.Setpoint = gtvt::real;
      EEPROM.put<type::Real>(
        GOS_TC_EEPROM_INDEX_SETPOINT,
        gtp::parameter.Setpoint);
      if (!displayed) {
        gatl::format::real<type::Real, uint8_t>(
          gtfdb::second,
          gtp::parameter.Setpoint,
          gtf::real::option,
          &gtfdb::text::setpoint,
          &gtfdb::text::unit::degree::centigrade);
        gt::variables::status = gt::type::Status::automatic;
        displayed = true;
      }
    }
  }
  if (location = gatl::modbus::access::buffer::location(
    gtm::variable,
    gtm::buffer::request,
    GOS_TC_HRA_INTERVAL,
    1,
    address,
    length)) {
    gt::variables::temporary::integer = MODBUS_READ_UINT16_AT0(location);
    if (gt::variables::temporary::integer != gt::variables::timing::interval) {
      gt::variables::timing::interval = gt::variables::temporary::integer;
      EEPROM.put<type::Unsigned>(GOS_TC_EEPROM_INDEX_INTERVAL, gtvi::interval);
      if (!displayed) {
        gatl::format::integer<type::Unsigned, uint8_t>(
          gtfdb::second,
          gt::variables::timing::interval,
          &gt::format::display::buffer::text::interval);
        displayed = true;
      }
    }
    result = MODBUS_STATUS_OK;
  }
  if (location = gatl::modbus::access::buffer::location<MODBUS_TYPE_DEFAULT>(
    gtm::variable,
    gtm::buffer::request,
    GOS_TC_HRA_KP,
    2,
    address,
    length)) {
    gtvt::real = gatlun::part::combine<uint16_t, type::Real>(
      MODBUS_READ_UINT16_AT0(location),
      MODBUS_READ_UINT16_AT0(location + 2));
    if (gtvt::real != gtp::parameter.Kp) {
      calculatetime = true;
      gtp::parameter.Kp = gtvt::real;
      EEPROM.put<type::Real>(
        GOS_TC_EEPROM_INDEX_KP,
        gtp::parameter.Kp);
      if (!displayed) {
        gatl::format::real<type::Real, uint8_t>(
          gtfdb::second,
          gtp::parameter.Kp,
          gtf::real::option,
          &gtfdb::text::kp);
        displayed = true;
      }
    }
  }
  if (location = gatl::modbus::access::buffer::location<MODBUS_TYPE_DEFAULT>(
    gtm::variable,
    gtm::buffer::request,
    GOS_TC_HRA_I,
    2,
    address,
    length)) {
    gtvt::real = gatlun::part::combine<uint16_t, type::Real>(
      MODBUS_READ_UINT16_AT0(location),
      MODBUS_READ_UINT16_AT0(location + 2));
    if (bitRead(gtv::modbus::coils, GOS_TCV_COIL_BIT_TUNE_TIME_MASTER)) {
      if (gtvt::real != gtp::tune::t.Ti) {
        calculatetime = true;
        gtp::tune::t.Ti = gtvt::real;
        EEPROM.put<type::Real>(GOS_TC_EEPROM_INDEX_KITI, gtp::tune::t.Ti);
        if (!displayed) {
          gatl::format::real<type::Real, uint8_t>(
            gtfdb::second,
            gtp::tune::t.Ti,
            gtf::real::option,
            &gtfdb::text::ti);
          displayed = true;
        }
      }
    } else {
      if (gtvt::real != gtp::tune::k.Ki) {
        calculatetime = true;
        gtp::tune::k.Ki = gtvt::real;
        EEPROM.put<type::Real>(GOS_TC_EEPROM_INDEX_KITI, gtp::tune::k.Ki);
        if (!displayed) {
          gatl::format::real<type::Real, uint8_t>(
            gtfdb::second,
            gtp::tune::k.Ki,
            gtf::real::option,
            &gtfdb::text::ki);
          displayed = true;
        }
      }
    }
    result = MODBUS_STATUS_OK;
  }
  if (location = gatl::modbus::access::buffer::location<MODBUS_TYPE_DEFAULT>(
    gtm::variable,
    gtm::buffer::request,
    GOS_TC_HRA_D,
    2,
    address,
    length)) {
    gtvt::real = gatlun::part::combine<uint16_t, type::Real>(
      MODBUS_READ_UINT16_AT0(location),
      MODBUS_READ_UINT16_AT0(location + 2));
    if (bitRead(gtv::modbus::coils, GOS_TCV_COIL_BIT_TUNE_TIME_MASTER)) {
      if (gtvt::real != gtp::tune::t.Td) {
        calculatetime = true;
        gtp::tune::t.Td = gtvt::real;
        EEPROM.put<type::Real>(GOS_TC_EEPROM_INDEX_KDTD, gtp::tune::t.Td);
        if (!displayed) {
          gatl::format::real<type::Real, uint8_t>(
            gtfdb::second,
            gtp::tune::t.Td,
            gtf::real::option,
            &gtfdb::text::td);
          displayed = true;
        }
      }
    } else {
      if (gtvt::real != gtp::tune::k.Kd) {
        calculatetime = true;
        gtp::tune::k.Kd = gtvt::real;
        EEPROM.put<type::Real>(GOS_TC_EEPROM_INDEX_KDTD, gtp::tune::k.Kd);
        if (!displayed) {
          gatl::format::real<type::Real, uint8_t>(
            gtfdb::second,
            gtp::tune::k.Kd,
            gtf::real::option,
            &gtfdb::text::kd);
          displayed = true;
        }
      }
    }
    result = MODBUS_STATUS_OK;
  }
  if (location = gatl::modbus::access::buffer::location<MODBUS_TYPE_DEFAULT>(
    gtm::variable,
    gtm::buffer::request,
    GOS_TC_HRA_MIN_SENS,
    2,
    address,
    length)) {
    gtvt::real = gatlun::part::combine<uint16_t, double>(
      MODBUS_READ_UINT16_AT0(location),
      MODBUS_READ_UINT16_AT0(location + 2));
    if (gtvt::real != gt::sensor::max6675sensor.Range.lowest) {
      gt::sensor::max6675sensor.Range.lowest = gtvt::real;
      EEPROM.put<double>(
        GOS_TC_EEPROM_INDEX_MIN_SENS,
        gt::sensor::max6675sensor.Range.lowest);
      if (!displayed) {
        gatl::format::real<type::Real, uint8_t>(
          gtfdb::second,
          gt::sensor::max6675sensor.Range.lowest,
          gtf::real::option,
          &gtfdb::text::minsens);
        displayed = true;
      }
    }
  }
  if (location = gatl::modbus::access::buffer::location<MODBUS_TYPE_DEFAULT>(
    gtm::variable,
    gtm::buffer::request,
    GOS_TC_HRA_MAX_SENS,
    2,
    address,
    length)) {
    gtvt::real = gatlun::part::combine<uint16_t, double>(
      MODBUS_READ_UINT16_AT0(location),
      MODBUS_READ_UINT16_AT0(location + 2));
    if (gtvt::real != gt::sensor::max6675sensor.Range.highest) {
      gt::sensor::max6675sensor.Range.highest = gtvt::real;
      EEPROM.put<double>(
        GOS_TC_EEPROM_INDEX_MIN_SENS,
        gt::sensor::max6675sensor.Range.highest);
      if (!displayed) {
        gatl::format::real<type::Real, uint8_t>(
          gtfdb::second,
          gt::sensor::max6675sensor.Range.highest,
          gtf::real::option,
          &gtfdb::text::maxsens);
        displayed = true;
      }
    }
  }
gos_modbus_handler_write_coils_finaly:
  if (displayed) {
    gt::display::two.display(gtfdb::first, gtfdb::second);
  }
  digitalWrite(PIN_LED_MODBUS_WRITE, LOW);
  return result;
}

MODBUS_TYPE_RESULT gtm::Handler::ReadExceptionStatus() {
  return MODBUS_STATUS_ILLEGAL_FUNCTION;
}

} // namespace modbus
} // namespace temperature
} // namespace gos
