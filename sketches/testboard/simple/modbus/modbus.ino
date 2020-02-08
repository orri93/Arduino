/*
 * SPI
 *                      UNO
 * 1 SLK <  SCK      Pin 13
 * 2 CS  <  D10      Pin 10
 * 3 DO  > MISO      Pin 12
 * 4 VCC    VCC      3.3/5V
 * 5 GND    GND         GND
 */

 /* Build Options */
//#define NO_DISPLAY

#include <SPI.h>

#include <gatled.h>
#include <gatleeprom.h>
#include <gatlmodbus.h>

#ifndef NO_DISPLAY
#include <gatldisplay.h>
#include <gatlformat.h>
#endif

#define GOS_MODBUS_DO_NOTHING

#define PIN_RS485_MODBUS_RX                  0
#define PIN_RS485_MODBUS_TX                  1
#define PIN_RS485_MODBUS_TE                  2

#define PIN_LED_RED_A                        3
#define PIN_LED_RED_B                       11

#define PIN_LED_BLUE_A                       6
#define PIN_LED_BLUE_B                       5

#define PIN_LED_GREEN                        9
#define PIN_LED_YELLOW                      10

#define PIN_LED_MODBUS_READ      PIN_LED_GREEN
#define PIN_LED_MODBUS_WRITE    PIN_LED_YELLOW

#define PIN_BUTTON_A                         8
#define PIN_BUTTON_B                         7

#define PIN_POTENTIOMETER                   A0

#define MODBUS_BAUD                       9600

#define MODBUS_SLAVE_ID                      1

#define MODBUS_BUFFER_SIZE                  64

#define GOS_BARRAY_BINDING

namespace gatl = ::gos::atl;
namespace gatlu = ::gos::atl::utility;
namespace gatll = ::gos::atl::led;

namespace gos {
namespace modbus {

namespace type {
typedef float Real;
typedef int16_t Signed;
typedef uint16_t Unsigned;
} /* End of type name-space */

namespace value {
namespace zero {
const type::Real Real = 0.0F;
const type::Signed Signed = 0;
const type::Unsigned Unsigned = 0;
}
} /* End of value name-space */

namespace variables {
namespace real {
type::Real first = value::zero::Real, second = value::zero::Real;
}
type::Signed signedv = value::zero::Signed;
type::Unsigned unsignedv = value::zero::Unsigned;
namespace temporary {
type::Unsigned integer;
type::Real real;
}
} /* End of variables name-space */

namespace potentiometer {
type::Unsigned value;
type::Real real;
}

namespace binding {
#ifdef GOS_BARRAY_BINDING
namespace barray {
gatl::binding::barray::reference<
  type::Real, MODBUS_TYPE_DEFAULT, MODBUS_TYPE_BIND_INDEX> real;
}
#else
gatl::binding::reference<type::Real, uint16_t> real;
#endif
void create();
}

namespace modbus {
void initialize();
class Handler : public virtual ::gos::atl::modbus::Handler<> {
public:
  MODBUS_TYPE_RESULT ReadCoils(
    const MODBUS_TYPE_FUNCTION& function,
    const MODBUS_TYPE_DEFAULT& address,
    const MODBUS_TYPE_DEFAULT& length);
  MODBUS_TYPE_RESULT ReadDiscretes(
    const MODBUS_TYPE_FUNCTION& function,
    const MODBUS_TYPE_DEFAULT& address,
    const MODBUS_TYPE_DEFAULT& length);
  MODBUS_TYPE_RESULT ReadHoldingRegisters(
    const MODBUS_TYPE_FUNCTION& function,
    const MODBUS_TYPE_DEFAULT& address,
    const MODBUS_TYPE_DEFAULT& length);
  MODBUS_TYPE_RESULT ReadInputRegisters(
    const MODBUS_TYPE_FUNCTION& function,
    const MODBUS_TYPE_DEFAULT& address,
    const MODBUS_TYPE_DEFAULT& length);
  MODBUS_TYPE_RESULT WriteCoils(
    const MODBUS_TYPE_FUNCTION& function,
    const MODBUS_TYPE_DEFAULT& address,
    const MODBUS_TYPE_DEFAULT& length);
  MODBUS_TYPE_RESULT WriteHoldingRegisters(
    const MODBUS_TYPE_FUNCTION& function,
    const MODBUS_TYPE_DEFAULT& address,
    const MODBUS_TYPE_DEFAULT& length);
  MODBUS_TYPE_RESULT ReadExceptionStatus(const MODBUS_TYPE_FUNCTION& function);
};

namespace binding {
void create();
} /* End of modbus binding name-space */
namespace buffer {
gatl::buffer::Holder<uint16_t, MODBUS_TYPE_BUFFER> request(MODBUS_BUFFER_SIZE);
gatl::buffer::Holder<uint16_t, MODBUS_TYPE_BUFFER> response(MODBUS_BUFFER_SIZE);
}
Handler handler;
gatl::modbus::structures::Parameter<> parameter;
gatl::modbus::structures::Variable<> variable;
} /* End of modbus name-space */

#ifndef NO_DISPLAY
namespace format {
namespace display {
namespace buffer {
gatl::buffer::Holder<> first;
gatl::buffer::Holder<> second;
}
namespace crc {
namespace last {
uint16_t first, second;
}
}
}
} /* End of format name-space */

namespace display {
gatl::display::Oled<> oled;
gatl::display::asynchronous::line::Two<> twoline(oled);
bool updated = false;
namespace update {
namespace second {
void line();
}
}
} /* End of display name-space */
#endif

namespace eeprom {
namespace binding {
namespace integer {
namespace index {
}
}
void read();
void update();
}
} /* End of eeprom name-space */

} /* End of modbus name-space */
} /* End of gos name-space */

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

#ifndef NO_DISPLAY
  gmd::oled.U8g2->begin();
#endif

  gm::modbus::initialize();

  /* RS485 */
  Serial.begin(MODBUS_BAUD);

  gatl::modbus::begin<>(
    Serial,
    gm::modbus::parameter,
    gm::modbus::variable,
    MODBUS_BAUD);

  gatl::buffer::clear(gm::format::display::buffer::first);
  gatl::buffer::clear(gm::format::display::buffer::second);

  gm::format::display::crc::last::first =
    gatl::utility::crc::calculate<>(gm::format::display::buffer::first);
  gm::format::display::crc::last::second =
    gatl::utility::crc::calculate<>(gm::format::display::buffer::second);
}


void loop() {

  gm::potentiometer::value = analogRead(PIN_POTENTIOMETER);
  gm::potentiometer::real = (gm::type::Real)(gm::potentiometer::value)/1023.0F;

#ifdef MODBUS_BAUD
  gatl::modbus::loop<uint16_t>(
    Serial,
    gm::modbus::parameter,
    gm::modbus::handler,
    gm::modbus::variable,
    gm::modbus::buffer::request,
    gm::modbus::buffer::response);
#endif

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

namespace display {
} /* End of display name-space */

namespace binding {
void create() {
}
}

namespace eeprom {
namespace binding {
void read() {
  //gatl::eeprom::read(gm::binding::barray::real);
}
void update() {
  //gatl::eeprom::update(gm::binding::barray::real);
}
}
} /* End of eeprom name-space */

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
namespace binding {
void create() {
}
}
} /* End of modbus name-space */

}
}
