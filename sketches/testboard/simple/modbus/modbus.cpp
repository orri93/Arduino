/*
 * SPI
 *                      UNO
 * 1 SLK <  SCK      Pin 13
 * 2 CS  <  D10      Pin 10
 * 3 DO  > MISO      Pin 12
 * 4 VCC    VCC      3.3/5V
 * 5 GND    GND         GND
 */

#include <gatled.h>
#include <gatlmodbus.h>

#include "type.h"
#include "value.h"
#include "variable.h"
#include "binding.h"
#include "macro.h"

namespace gatl = ::gos::atl;
namespace gatlu = ::gos::atl::utility;
namespace gatll = ::gos::atl::led;

namespace gm = ::gos::modbus;
namespace gmt = ::gos::modbus::type;
namespace gme = ::gos::modbus::eeprom;
namespace gmv = ::gos::modbus::variables;
namespace gmeb = ::gos::modbus::eeprom::binding;

#ifndef NO_DISPLAY
namespace gmd = ::gos::modbus::display;
#endif

void setup() {
  gm::binding::create();
  gm::modbus::binding::create();
  gme::binding::read();

  gatll::initialize(PIN_LED_RED_A);
  gatll::initialize(PIN_LED_RED_B);

  gatll::initialize(PIN_LED_BLUE_A);
  gatll::initialize(PIN_LED_BLUE_B);

  gatll::initialize(PIN_LED_GREEN);
  gatll::initialize(PIN_LED_YELLOW);

  gatll::blink(PIN_LED_MODBUS_READ);
  gatll::blink(PIN_LED_MODBUS_WRITE);

  pinMode(PIN_BUTTON_A, INPUT_PULLUP);
  pinMode(PIN_BUTTON_B, INPUT_PULLUP);

  gm::modbus::initialize();

  /* RS485 */
  Serial.begin(MODBUS_BAUD);

  gatl::modbus::begin<>(
    Serial,
    gm::modbus::parameter,
    gm::modbus::variable,
    MODBUS_BAUD);

#ifndef NO_DISPLAY
  gmd::oled.U8g2->begin();

  gatl::buffer::clear(gm::format::display::buffer::first);
  gatl::buffer::clear(gm::format::display::buffer::second);

  gm::format::display::crc::last::first =
    gatl::utility::crc::calculate<>(gm::format::display::buffer::first);
  gm::format::display::crc::last::second =
    gatl::utility::crc::calculate<>(gm::format::display::buffer::second);
#endif
}


void loop() {
  gatl::modbus::loop<uint16_t>(
    Serial,
    gm::modbus::parameter,
    gm::modbus::handler,
    gm::modbus::variable,
    gm::modbus::buffer::request,
    gm::modbus::buffer::response);

  gm::variables::temporary::integer =
    gatl::utility::crc::calculate<>(gm::format::display::buffer::first);
  if (
    gm::variables::temporary::integer !=
    gm::format::display::crc::last::first) {
    gm::display::updated = true;
    gm::format::display::crc::last::first = gm::variables::temporary::integer;
  }
  gm::variables::temporary::integer =
    gatl::utility::crc::calculate<>(gm::format::display::buffer::second);
  if (gm::variables::temporary::integer !=
    gm::format::display::crc::last::second) {
    gm::display::updated = true;
    gm::format::display::crc::last::second = gm::variables::temporary::integer;
  }

#ifndef NO_DISPLAY
  if (gm::display::updated) {
    gm::display::twoline.display(
      gm::format::display::buffer::first,
      gm::format::display::buffer::second);
    gm::display::updated = false;
  }
  gm::display::twoline.loop();
#endif
}


namespace gos {
namespace modbus {

void initialize() {
  parameter.Id = 1;
  parameter.Control = PIN_RS485_MODBUS_TE;
}

/* 0x01 Read Coils */
uint8_t gm::modbus::Handler::ReadCoils(
  const uint8_t& function,
  const uint16_t& start,
  const uint16_t& length) {
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
uint8_t gm::modbus::Handler::ReadDiscretes(
  const uint8_t& function,
  const uint16_t& start,
  const uint16_t& length) {
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  bool bs;
  uint8_t result = MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  if(gatl::utility::range::ismemberof<uint16_t>(0x0000, start, length)) {
    bs = digitalRead(PIN_BUTTON_A) == LOW;
    result = gatl::modbus::provide::discrete<uint16_t>(
      gm::modbus::variable,
      gm::modbus::buffer::request,
      gm::modbus::buffer::response,
      0x0000,
      bs);
    if (result == MODBUS_STATUS_OK) {
      gatl::buffer::strncpy(
        gm::format::display::buffer::first,
        (bs ? TEXT_DISCR_0_IS_1 : TEXT_DISCR_0_IS_0));
    } else {
      gatl::buffer::strncpy(
        gm::format::display::buffer::first,
        TEXT_DISCR_0_IS_E);
      digitalWrite(PIN_LED_MODBUS_READ, LOW);
      return result;
    }
  }
  if (gatl::utility::range::ismemberof<uint16_t>(0x0001, start, length)) {
    bs = digitalRead(PIN_BUTTON_B) == LOW;
    result = gatl::modbus::provide::discrete<uint16_t>(
      gm::modbus::variable,
      gm::modbus::buffer::request,
      gm::modbus::buffer::response,
      0x0000,
      bs);
    if (result == MODBUS_STATUS_OK) {
      gatl::buffer::strncpy(
        gm::format::display::buffer::second,
        (bs ? TEXT_DISCR_1_IS_1 : TEXT_DISCR_1_IS_0));
    } else {
      gatl::buffer::strncpy(
        gm::format::display::buffer::second,
        TEXT_DISCR_1_IS_E);
      digitalWrite(PIN_LED_MODBUS_READ, LOW);
      return result;
    }
  }
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return result;
}

/* 0x03 Read Multiple Holding Registers */
uint8_t gm::modbus::Handler::ReadHoldingRegisters(
  const uint8_t& function,
  const uint16_t& start,
  const uint16_t& length) {
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
MODBUS_TYPE_RESULT gm::modbus::Handler::ReadInputRegisters(
  const MODBUS_TYPE_FUNCTION& function,
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
uint8_t gm::modbus::Handler::WriteCoils(
  const uint8_t& function,
  const uint16_t& start,
  const uint16_t& length) {
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
uint8_t gm::modbus::Handler::WriteHoldingRegisters(
  const uint8_t& function,
  const uint16_t& start,
  const uint16_t& length) {
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
MODBUS_TYPE_RESULT gm::modbus::Handler::ReadExceptionStatus(
  const MODBUS_TYPE_FUNCTION& function) {
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return MODBUS_STATUS_OK;
}

}
}
