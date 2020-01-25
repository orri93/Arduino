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

#include <gatleeprom.h>
#include <gatlmodbus.h>

#ifndef NO_DISPLAY
#include <gatldisplay.h>
#include <gatlformat.h>
#endif

#define PIN_RS485_MODBUS_RX                  0
#define PIN_RS485_MODBUS_TX                  1
#define PIN_RS485_MODBUS_TE                  2

#define MODBUS_BAUD                       9600

#define MODBUS_SLAVE_ID                      1

#define MODBUS_BUFFER_SIZE                  64

#define GOS_BARRAY_BINDING

namespace gatl = ::gos::atl;
namespace gatlu = ::gos::atl::utility;

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
}

namespace variables {
namespace real {
type::Real first = value::zero::Real, second = value::zero::Real;
}
type::Signed signedv = value::zero::Signed;
type::Unsigned unsignedv = value::zero::Unsigned;
namespace temporary {
uint16_t uint16;
}
} /* End of variables name-space */

namespace binding {
namespace index {
namespace real {
const uint8_t First = 1;
const uint8_t Second = 2;
}
}
namespace count {
const uint8_t Real = 2;
}
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

namespace variables {
type::Real setpoint = value::zero::Real;
}

namespace binding {
namespace count {
const uint8_t Coil = 1;
namespace input {
const uint8_t Output = 1;
const uint8_t Sensor = 1;
}
}
namespace index {
namespace coil {
const uint8_t UseT = 0;
}
namespace input {
const uint8_t Output = 0;
const uint8_t Temperature = 0;
}
}
gatl::binding::reference<bool, uint16_t> coils;
namespace input {
gatl::binding::reference<type::Output, uint16_t> output;
gatl::binding::reference<::gos::modbus::type::Real, uint16_t> sensor;
}
void create();
} /* End of modbus binding name-space */
namespace buffer {
gatl::buffer::Holder<uint16_t, char> request(MODBUS_BUFFER_SIZE);
gatl::buffer::Holder<uint16_t, char> response(MODBUS_BUFFER_SIZE);
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
namespace index {
namespace boolean {
const uint8_t UseT = 0;
}
}
namespace count {
const uint8_t boolean = 1;
}
gatl::binding::reference<bool, int> boolean;
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

#ifndef NO_DISPLAY
  gmd::oled.U8g2->begin();
  gm::format::display::option::tuning.Precision = 3;
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
  gm::variables::tick = millis();

#ifdef MODBUS_BAUD
  gatl::modbus::loop<uint16_t>(
    Serial,
    gm::modbus::parameter,
    gm::modbus::handler,
    gm::modbus::variable,
    gm::modbus::buffer::request,
    gm::modbus::buffer::response);
#endif

  uint16 = gatl::utility::crc::calculate<>(gm::format::display::buffer::first);
  if (uint16 != gm::format::display::crc::last::first) {
    gm::display::updated = true;
    gm::format::display::crc::last::first = uint16;
  }
  uint16 = gatl::utility::crc::calculate<>(gm::format::display::buffer::second);
  if (uint16 != gm::format::display::crc::last::second) {
    gm::display::updated = true;
    gm::format::display::crc::last::second = uint16;
  }

  if (gm::timer::cycle.is(gmv::tick)) {
    gm::display::update::second::line();
    gm::sensor::read();
    if (gm::mode::state == gm::mode::status::automatic) {
      gm::variables::output = static_cast<gm::type::Output>(
        gm::variables::tick % 255);
    }
  }

#ifndef MODBUS_BAUD
  Serial.print(gmv::temperature);
  Serial.print(",");
  Serial.print(gm::variables::output);
  Serial.println();
#endif

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

namespace sensor {
void read() {
  error::message = nullptr;
  if (max6675.read(value)) {
    if (::gos::sensor::range::check(
      value,
      SENSOR_MINIMUM,
      SENSOR_MAXIMUM) == GOS_SENSOR_STATUS_OK) {
      gmv::temperature = static_cast<gm::type::Real>(value);
    } else {
      error::message = ::gos::sensor::error(error::length);
    }
  } else {
    error::message = max6675.error(error::length);
  }
  if (gm::sensor::error::message == nullptr) {
    gatl::format::real(
      gm::format::display::buffer::first,
      gmv::temperature,
      gm::format::display::option::temperature,
      &gm::format::display::buffer::id::temperature,
      &gm::format::display::buffer::unit);
  } else {
    gatl::format::message(
      gm::format::display::buffer::first,
      gm::sensor::error::message,
      gm::sensor::error::length,
      &gm::format::display::buffer::id::temperature);
  }
  gm::display::updated = true;
}
} /* End of sensor name-space */

namespace binding {
void create() {
  /*
     *  40001  Manual     (0x0000)
     *  40002  Setpoint   (0x0001)
     *  40003  --L--
     *  40004  Kp         (0x0003)
     *  40005  --L--
     *  40006  Ki         (0x0005)
     *  40007  --L--
     *  40008  Kd         (0x0007)
     *  40009  --L--
     *  40010  Ti         (0x0009)
     *  40011  --L--
     *  40012  Td         (0x000b)
     *  40013  --L--
     */
  gm::variables::temporary::address =
    gatl::binding::barray::create<gm::type::Output, uint16_t, uint8_t>(
      gm::binding::barray::output,
      0,
      gm::binding::count::Output,
      sizeof(gm::type::Output));
  gatl::binding::barray::create<gm::type::Real, uint16_t, uint8_t>(
    gm::binding::barray::real,
    gm::variables::temporary::address,
    gm::binding::count::Real,
    sizeof(gm::type::Real));
}
}

namespace eeprom {
namespace binding {
void read() {
  gatl::eeprom::read(gm::binding::barray::real);
}
void update() {
  gatl::eeprom::update(gm::binding::barray::real);
}
}
} /* End of eeprom name-space */

#ifdef MODBUS_BAUD
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

#ifdef GOS_TODO_UPGRADE
  gmvt::status = MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  if (gatl::modbus::binding::coil::access(binding::coils, slave, start, length)) {
    gmvt::status = STATUS_OK;
  }
  return gmvt::status;
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

/* 0x03 Read Multiple Holding Registers */
uint8_t gm::modbus::Handler::ReadHoldingRegisters(
  const uint8_t& function,
  const uint16_t& start,
  const uint16_t& length) {
#ifdef GOS_TODO_UPGRADE
  gmvt::status = MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  if (gatl::modbus::binding::registers::access(
    binding::holding::manual,
    slave,
    startaddress,
    length)) {
    gmvt::status = STATUS_OK;
  }
  if (gatl::modbus::binding::two::access(
    binding::holding::pid,
    slave,
    startaddress,
    length)) {
    gmvt::status = STATUS_OK;
  }
  return gmvt::status;
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
uint8_t gm::modbus::Handler::ReadInputRegisters(
  const uint8_t& function,
  const uint16_t& start,
  const uint16_t& length) {
#ifdef GOS_TODO_UPGRADE
  gmvt::status = MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  if (gatl::modbus::binding::registers::access(
    binding::input::output,
    slave,
    startaddress,
    length)) {
    gmvt::status = STATUS_OK;
  }
  if (gatl::modbus::binding::two::access(
    binding::input::sensor,
    slave,
    startaddress,
    length)) {
    gmvt::status = STATUS_OK;
  }
  return gmvt::status;
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
#ifdef GOS_TODO_UPGRADE
  gmvt::status = MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  if (gatl::modbus::binding::coil::assign(
    binding::coils,
    slave,
    startaddress,
    length,
    gmvt::index,
    gmvt::to)) {
    gmvt::status = STATUS_OK;
  }
  return gmvt::status;
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
#ifdef GOS_TODO_UPGRADE
  if (gatl::modbus::binding::registers::assign(
    binding::holding::manual,
    slave,
    startaddress,
    length,
    gmvt::index,
    gmvt::to)) {
    gmvt::status = STATUS_OK;
    while (gmvt::index < gmvt::to) {
      if (gmvt::index == binding::index::holding::Manual) {
        if (gatl::utility::range::isinside(
          variables::manual,
          gm::variables::range::output)) {
          if (gatlu::changed::apply::is<type::Output>(
            variables::manual,
            variables::last::manual)) {
            heater::manual = variables::manual;
            if (gm::mode::is::unequal(mode::status::manual)) {
              mode::gotom::state(mode::status::manual);
            }
          }
        } else {
          return STATUS_ILLEGAL_DATA_VALUE;
        }
      }
      gmvt::index++;
    }
  }
  if (gatl::modbus::binding::two::assign(
    binding::holding::pid,
    slave,
    startaddress,
    length,
    gmvt::index,
    gmvt::to)) {
    gmvt::status = STATUS_OK;
    gm::variables::temporary::is = false;
    while (gmvt::index < gmvt::to) {
      if (gatl::binding::change::is(binding::holding::pid, gmvt::index)) {
        switch (gmvt::index) {
        case GOS_MP_PID_HOLDING_SP_INDEX:
          if (gatl::utility::range::isinside(
            variables::setpoint,
            gm::variables::range::setpoint)) {
            if (gatlu::changed::apply::is<type::Real>(
              variables::setpoint,
              variables::last::setpoint)) {
              pid::parameter.Setpoint = variables::setpoint;
              if (gm::mode::is::unequal(mode::status::automatic)) {
                mode::gotom::state(mode::status::automatic);
              }
            }
          } else {
            return STATUS_ILLEGAL_DATA_VALUE;
          }
          break;
        case GOS_MP_PID_HOLDING_KP_INDEX:
          gm::variables::temporary::is = true;
          gm::mode::gotom::state(gm::mode::status::kp);
          break;
        case GOS_MP_PID_HOLDING_KI_INDEX:
          if (!gm::variables::options::uset) {
            gm::variables::temporary::is = true;
            gm::mode::gotom::state(gm::mode::status::ki);
          }
          break;
        case GOS_MP_PID_HOLDING_KD_INDEX:
          if (!gm::variables::options::uset) {
            gm::variables::temporary::is = true;
            gm::mode::gotom::state(gm::mode::status::kd);
          }
          break;
        case GOS_MP_PID_HOLDING_TI_INDEX:
          if (gm::variables::options::uset) {
            gm::variables::temporary::is = true;
            gm::mode::gotom::state(gm::mode::status::ti);
          }
          break;
        case GOS_MP_PID_HOLDING_TD_INDEX:
          if (gm::variables::options::uset) {
            gm::variables::temporary::is = true;
            gm::mode::gotom::state(gm::mode::status::td);
          }
          break;
        }
      }
      gmvt::index++;
    }
    if (gm::variables::temporary::is) {
      gm::pid::tune::apply();
    }
  }
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
  return MODBUS_STATUS_OK;
}
namespace binding {
namespace size {
const uint8_t Output = 1;
const uint8_t Real = 2;
}
void create() {
  /*
   *  00001  Use T tuning values
   *
   */
  gmvt::address = gatl::binding::create<bool, uint16_t, uint8_t>(
    binding::coils,
    0,
    binding::count::Coil,
    1);
  gatl::binding::set<bool, uint16_t, uint8_t>(
    binding::coils,
    index::coil::UseT,
    &gm::variables::options::uset);

  /*
   *  30001  Output from PID
   *  30002  Temperature
   *  30003  --L--
   */
  gmvt::address = gatl::binding::create<gm::type::Output, uint16_t, uint8_t>(
    binding::input::output,
    0,
    binding::count::input::Output,
    size::Output);
  gatl::binding::set<gm::type::Output, uint16_t, uint8_t>(
    binding::input::output,
    index::input::Output,
    &gm::variables::output);

  gmvt::address = gatl::binding::create<gm::type::Real, uint16_t, uint8_t>(
    binding::input::sensor,
    gmvt::address,
    binding::count::input::Sensor,
    size::Real);
  gatl::binding::set<gm::type::Real, uint16_t, uint8_t>(
    binding::input::sensor,
    index::input::Temperature,
    &gm::variables::temperature);
}
}
} /* End of modbus name-space */
#endif

}
}
