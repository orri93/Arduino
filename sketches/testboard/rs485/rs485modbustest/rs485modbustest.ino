/*
 * SPI
 *                      UNO
 * 1 SLK <  SCK      Pin 13
 * 2 CS  <  D10      Pin 10
 * 3 DO  > MISO      Pin 12
 * 4 VCC    VCC      3.3/5V
 * 5 GND    GND         GND
 *
 */

 /* Build Options */
//#define NO_DISPLAY

#include <SPI.h>

#include <gosmax6675.h>
#include <arduinosensor.h>
#include <arduinotick.h>

#include <gatleeprom.h>
#include <gatlmodbus.h>

#include <gatldisplay.h>
#include <gatlformat.h>

#define PIN_RS485_MODBUS_RX                  0
#define PIN_RS485_MODBUS_TX                  1
#define PIN_RS485_MODBUS_TE                  2
#define PIN_MAX6675_CS                       8

#define INTERVAL_CYCLE                    1000

#define DELAY_SENSOR_SETUP_END             500

#define SERIAL_BAUD                      19200
#define MODBUS_BAUD                       9600

#define MODBUS_SLAVE_ID                      1

#define MODBUS_BUFFER_SIZE                  64

#define SENSOR_MINIMUM                       0
#define SENSOR_MAXIMUM                     500

#define SETPOINT_MININUM                     0
#define SETPOINT_MAXIMUM                   300

#define PID_MINIMUM_OUTPUT                   0
#define PID_MAXIMUM_OUTPUT                 255

#define TEXT_ID_TEMPERATURE "T: "
#define TEXT_ID_SETPOINT "S: "
#define TEXT_ID_MANUAL "M: "
#define TEXT_ID_IDLE "I: "
#define TEXT_ID_KP "KP: "
#define TEXT_ID_KI "KI: "
#define TEXT_ID_KD "KD: "
#define TEXT_ID_TI "TI: "
#define TEXT_ID_TD "TD: "

#define TEXT_UNIT " C"

#define GOS_MP_PID_HOLDING_SP_INDEX 0
#define GOS_MP_PID_HOLDING_KP_INDEX 1
#define GOS_MP_PID_HOLDING_KI_INDEX 2
#define GOS_MP_PID_HOLDING_KD_INDEX 3
#define GOS_MP_PID_HOLDING_TI_INDEX 4
#define GOS_MP_PID_HOLDING_TD_INDEX 5

#define GOS_MP_PID_HOLDING_SP_ADDRESS  1
#define GOS_MP_PID_HOLDING_KP_ADDRESS  3
#define GOS_MP_PID_HOLDING_KI_ADDRESS  5
#define GOS_MP_PID_HOLDING_KD_ADDRESS  7
#define GOS_MP_PID_HOLDING_TI_ADDRESS  9
#define GOS_MP_PID_HOLDING_TD_ADDRESS 11

#define GOS_BARRAY_BINDING

namespace gatl = ::gos::atl;
namespace gatlu = ::gos::atl::utility;

namespace gos {
namespace modbus {

namespace mode {
enum class status { idle, coil, manual, automatic, kp, ki, kd, ti, td };
status state = status::idle;
namespace is {
bool equal(const status& state);
bool unequal(const status& state);
}
namespace gotom {
void next();
void state(const status& state);
}
} /* End of mode name-space */

namespace type {
typedef float Real;
typedef int16_t Output;
namespace optional {
typedef gatl::type::optional<type::Real> Real;
typedef gatl::type::optional<type::Output> Output;
}
} /* End of type name-space */

namespace value {
namespace zero {
const type::Real Real = 0.0F;
const type::Output Output = 0;
}
}

namespace variables {
type::Real temperature = value::zero::Real;
type::Output output = value::zero::Output;
unsigned long tick;
namespace temporary {
int address;
uint8_t status;
uint8_t index, to;
bool is, ist, isk;
}
namespace range {
gatl::type::range<type::Real> setpoint = gatl::type::make_range<type::Real>(
  SETPOINT_MININUM, SETPOINT_MAXIMUM);
gatl::type::range<type::Output> output = gatl::type::make_range<type::Output>(
  PID_MINIMUM_OUTPUT, PID_MAXIMUM_OUTPUT);
}
namespace options {
bool uset = false;
}
} /* End of variables name-space */

namespace binding {
namespace index {
namespace real {
const uint8_t Setpoint = GOS_MP_PID_HOLDING_SP_INDEX;
const uint8_t Kp = GOS_MP_PID_HOLDING_KP_INDEX;
const uint8_t Ki = GOS_MP_PID_HOLDING_KI_INDEX;
const uint8_t Kd = GOS_MP_PID_HOLDING_KD_INDEX;
const uint8_t Ti = GOS_MP_PID_HOLDING_TI_INDEX;
const uint8_t Td = GOS_MP_PID_HOLDING_TD_INDEX;
}
const uint8_t output = 0;
}
namespace count {
const uint8_t Output = 1;
const uint8_t Real = 6;
}
#ifdef GOS_BARRAY_BINDING
namespace barray {
::gos::atl::binding::barray::reference<type::Output, uint16_t, uint8_t> output;
gatl::binding::barray::reference<type::Real, uint16_t> real;
}
#else
gatl::binding::reference<type::Output, uint16_t> output;
gatl::binding::reference<type::Real, uint16_t> real;
#endif
void create();
}

#ifdef MODBUS_BAUD
namespace modbus {
typedef ::gos::atl::modbus::Handler<
  uint16_t,
  uint16_t,
  uint16_t,
  uint8_t> Base;
void initialize();
class Handler : public virtual Base {
public:
  Result ReadCoils(
    const Function& function,
    const Address& address,
    const Length& length);
  Result ReadHoldingRegisters(
    const Function& function,
    const Address& address,
    const Length& length);
  Result ReadInputRegisters(
    const Function& function,
    const Address& address,
    const Length& length);
  Result WriteCoils(
    const Function& function,
    const Address& address,
    const Length& length);
  Result WriteHoldingRegisters(
    const Function& function,
    const Address& address,
    const Length& length);
  Result ReadExceptionStatus(const Function& function);
};
namespace variables {
type::Output manual = value::zero::Output;
type::Real setpoint = value::zero::Real;
namespace last {
type::optional::Output manual;
type::optional::Real setpoint;
}
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
#endif

namespace format {
namespace display {
namespace option {
gatl::format::option::Number temperature;
gatl::format::option::Number tuning;
}
namespace buffer {
gatl::buffer::Holder<> first;
gatl::buffer::Holder<> second;
namespace id {
gatl::buffer::Holder<> temperature(TEXT_ID_TEMPERATURE, sizeof(TEXT_ID_TEMPERATURE));
gatl::buffer::Holder<> idle(TEXT_ID_IDLE, sizeof(TEXT_ID_IDLE));
gatl::buffer::Holder<> manual(TEXT_ID_MANUAL, sizeof(TEXT_ID_MANUAL));
gatl::buffer::Holder<> setpoint(TEXT_ID_SETPOINT, sizeof(TEXT_ID_SETPOINT));
namespace tune {
gatl::buffer::Holder<> kp(TEXT_ID_KP, sizeof(TEXT_ID_KP));
gatl::buffer::Holder<> ki(TEXT_ID_KI, sizeof(TEXT_ID_KI));
gatl::buffer::Holder<> kd(TEXT_ID_KD, sizeof(TEXT_ID_KD));
gatl::buffer::Holder<> ti(TEXT_ID_TI, sizeof(TEXT_ID_TI));
gatl::buffer::Holder<> td(TEXT_ID_TD, sizeof(TEXT_ID_TD));
}
}
gatl::buffer::Holder<> unit(TEXT_UNIT, sizeof(TEXT_UNIT));
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


namespace timer {
Tick cycle(INTERVAL_CYCLE);
#ifdef INTERVAL_INTERFACE
Tick interfaces(INTERVAL_INTERFACE);
#endif
} /* End of timer name-space */

namespace sensor {
double value;
uint8_t status;
::gos::Max6675 max6675(PIN_MAX6675_CS);
void read();
namespace error {
const char* message;
uint8_t length;
}
} /* End of sensor name-space */

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
namespace gmd = ::gos::modbus::display;
namespace gmv = ::gos::modbus::variables;
namespace gmeb = ::gos::modbus::eeprom::binding;
namespace gmvt = ::gos::modbus::variables::temporary;

void setup() {
  gm::binding::create();
#ifdef MODBUS_BAUD
  gm::modbus::binding::create();
#endif
  gme::binding::read();

#ifndef NO_DISPLAY
  gmd::oled.U8g2->begin();
#ifdef USE_MEDIAN
  displaybuffermedian.Buffer[0] = '\0';
#endif
  gm::format::display::option::tuning.Precision = 3;
#endif

  SPI.begin();

  gm::sensor::max6675.initialize();

#ifdef MODBUS_BAUD
  gm::modbus::initialize();

  /* RS485 */
  pinMode(PIN_RS485_MODBUS_TE, OUTPUT);
  Serial.begin(MODBUS_BAUD);

  gatl::modbus::begin<>(
    Serial,
    gm::modbus::parameter,
    gm::modbus::variable,
    MODBUS_BAUD);
#else
  Serial.begin(SERIAL_BAUD);
#endif

#ifdef DELAY_SENSOR_SETUP_END
  delay(DELAY_SENSOR_SETUP_END);
#endif
}

void loop() {
  gm::variables::tick = millis();

#ifdef MODBUS_BAUD
  gatl::modbus::loop<>(
    Serial,
    gm::modbus::parameter,
    dynamic_cast<::gos::modbus::modbus::Base&>(gm::modbus::handler),
    gm::modbus::variable,
    gm::modbus::buffer::request,
    gm::modbus::buffer::response);
#endif

  if (gm::mode::state == gm::mode::status::manual) {
    gm::variables::output = gm::modbus::variables::manual;
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

namespace mode {
namespace is {
bool equal(const status& state) {
  return ::gos::modbus::mode::state == state;
}
bool unequal(const status& state) {
  return ::gos::modbus::mode::state != state;
}
}
namespace gotom {
namespace details {
status next(const status& state) {
  switch (state) {
  case status::idle:
    return status::manual;
  case status::manual:
    return status::automatic;
  case status::automatic:
    return status::kp;
  case status::kp:
    return status::ki;
  case status::ki:
    return status::kd;
  case status::kd:
    return status::ti;
  case status::ti:
    return status::td;
  default:
    return status::idle;
  }
}
}
void next() {
  gotom::state(details::next(gm::mode::state));
}
void state(const status& state) {
  gm::mode::state = state;
}

}
} /* End of mode name-space */

namespace display {
namespace update {
namespace second {
void line() {
  switch (gm::mode::state) {
  case gm::mode::status::idle:
    gatl::format::real(
      gm::format::display::buffer::second,
      gatl::binding::barray::get<gm::type::Real, uint16_t>(
        gm::binding::barray::real, gm::binding::index::real::Setpoint),
      gm::format::display::option::temperature,
      &gm::format::display::buffer::id::idle,
      &gm::format::display::buffer::unit);
    break;
  case gm::mode::status::coil:

    break;
  case gm::mode::status::manual:
    gatl::format::integer(
      gm::format::display::buffer::second,
      gatl::binding::barray::get<gm::type::Output, uint16_t>(
        gm::binding::barray::output, gm::binding::index::output),
      &gm::format::display::buffer::id::manual);
    break;
  case gm::mode::status::automatic:
    gatl::format::real(
      gm::format::display::buffer::second,
      gatl::binding::barray::get<gm::type::Real, uint16_t>(
        gm::binding::barray::real, gm::binding::index::real::Setpoint),
      gm::format::display::option::temperature,
      &gm::format::display::buffer::id::setpoint,
      &gm::format::display::buffer::unit);
    break;
  case gm::mode::status::kp:
    gatl::format::real(
      gm::format::display::buffer::second,
      gatl::binding::barray::get<gm::type::Real, uint16_t>(
        gm::binding::barray::real, gm::binding::index::real::Kp),
      gm::format::display::option::tuning,
      &gm::format::display::buffer::id::tune::kp);
    break;
  case gm::mode::status::ki:
    gatl::format::real(
      gm::format::display::buffer::second,
      gatl::binding::barray::get<gm::type::Real, uint16_t>(
        gm::binding::barray::real, gm::binding::index::real::Ki),
      gm::format::display::option::tuning,
      &gm::format::display::buffer::id::tune::ki);
    break;
  case gm::mode::status::kd:
    gatl::format::real(
      gm::format::display::buffer::second,
      gatl::binding::barray::get<gm::type::Real, uint16_t>(
        gm::binding::barray::real, gm::binding::index::real::Kd),
      gm::format::display::option::tuning,
      &gm::format::display::buffer::id::tune::kd);
    break;
  case gm::mode::status::ti:
    gatl::format::real(
      gm::format::display::buffer::second,
      gatl::binding::barray::get<gm::type::Real, uint16_t>(
        gm::binding::barray::real, gm::binding::index::real::Ti),
      gm::format::display::option::tuning,
      &gm::format::display::buffer::id::tune::ti);
    break;
  case gm::mode::status::td:
    gatl::format::real(
      gm::format::display::buffer::second,
      gatl::binding::barray::get<gm::type::Real, uint16_t>(
        gm::binding::barray::real, gm::binding::index::real::Td),
      gm::format::display::option::tuning,
      &gm::format::display::buffer::id::tune::td);
    break;
  }
  gm::display::updated = true;
}
}
}
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
  parameter.SlaveId = 1;
  parameter.TransmissionControl = PIN_RS485_MODBUS_TE;
}

/* 0x01 Read Coils */
Handler::Result gm::modbus::Handler::ReadCoils(
  const Function& function,
  const Address& start,
  const Length& length) {
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
Handler::Result gm::modbus::Handler::ReadHoldingRegisters(
  const Function& function,
  const Address& start,
  const Length& length) {
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
  } else if (ro == ::gos::atl::modbus::binding::result::excluded &&
    rr == ::gos::atl::modbus::binding::result::excluded) {
    return MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  } else {
    return MODBUS_STATUS_SLAVE_DEVICE_FAILURE;
  }
#endif
}
/* 0x04 Read Input Registers */
Handler::Result gm::modbus::Handler::ReadInputRegisters(
  const Function& function,
  const Address& start,
  const Length& length) {
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
    gatl::modbus::binding::registers::access<>(
      gm::modbus::binding::input::output,
      gm::modbus::variable,
      gm::modbus::buffer::request,
      gm::modbus::buffer::response,
      start,
      length);
  ::gos::atl::modbus::binding::result rr =
    gatl::modbus::binding::two::access<>(
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
Handler::Result gm::modbus::Handler::WriteCoils(
  const Function& function,
  const Address& start,
  const Length& length) {
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
  uint8_t index;
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
Handler::Result gm::modbus::Handler::WriteHoldingRegisters(
  const Function& function,
  const Address& start,
  const Length& length) {
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
  uint8_t index;
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
Handler::Result gm::modbus::Handler::ReadExceptionStatus(
  const Function& function) {
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
