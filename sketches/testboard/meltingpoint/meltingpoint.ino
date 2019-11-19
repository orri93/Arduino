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

#include <ModbusSlave.h>

#include <gosmax6675.h>
#include <arduinosensor.h>
#include <arduinotick.h>

#include <gatltype.h>
#include <gatlutility.h> 

#include <gatlbinding.h>
#include <gatleeprom.h>
#include <gatlmodbus.h>

#include <gatlpid.h>

/* Median for push button */
#include <gatlmedian.h>

#include <gatldisplay.h>
#include <gatlformat.h>

namespace gatl = ::gos::atl;
namespace gatlu = ::gos::atl::utility;

#define PIN_RS485_MODBUS_RX                  0
#define PIN_RS485_MODBUS_TX                  1
#define PIN_RS485_MODBUS_TE                  2
#define PIN_HEATER                           6
#define PIN_MAX6675_CS                       8
#define PIN_PUSH_BUTTON                     A0
#define PIN_POTENTIOMETER                   A2

#define INTERVAL_INTERFACE                 250
#define INTERVAL_CYCLE                    1000

#define DELAY_SENSOR_SETUP_END             500

#define SERIAL_BAUD                      19200
#define MODBUS_BAUD                       9600

#define MODBUS_SLAVE_ID                      1 

#define PUSH_BUTTON_ANALOG_THREASHOLD      512
#define PUSH_BUTTON_LONG_PRESS_TIME        250
#define PUSH_BUTTON_REST_TIME               25

#define SENSOR_MINIMUM                       0
#define SENSOR_MAXIMUM                     500

#define POTENTIOMETER_RAW_MININUM            0
#define POTENTIOMETER_RAW_MAXIMUM         1024

#define SETPOINT_MININUM       0
#define SETPOINT_MAXIMUM     300

#define PID_MINIMUM_OUTPUT                   0
#define PID_MAXIMUM_OUTPUT                 255


#define TEXT_ID_TEMPERATURE "T: "
#define TEXT_ID_IDLE "I: "
#define TEXT_ID_SETPOINT "S: "
#define TEXT_ID_MANUAL "M: "
#define TEXT_ID_KP "Kp: "
#define TEXT_ID_KI "Ki: "
#define TEXT_ID_KD "Kd: "
#define TEXT_ID_TI "Ti: "
#define TEXT_ID_TD "Td: "
#define TEXT_ID_SET_KP "KP: "
#define TEXT_ID_SET_KI "KI: "
#define TEXT_ID_SET_KD "KD: "
#define TEXT_ID_SET_TI "TI: "
#define TEXT_ID_SET_TD "TD: "

#define TEXT_UNIT " C"

namespace gos {
namespace meltingpoint {

namespace mode {
enum class status { idle, manual, automatic, kp, ki, kd, ti, td };
status state = status::idle;
namespace is {
bool setting = false;
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
}
namespace range {
gatl::type::range<type::Real> setpoint = gatl::type::make_range<type::Real>(
  SETPOINT_MININUM, SETPOINT_MAXIMUM);
gatl::type::range<type::Output> output = gatl::type::make_range<type::Output>(
  PID_MINIMUM_OUTPUT, PID_MAXIMUM_OUTPUT);
}
} /* End of variables name-space */

#ifdef MODBUS_BAUD
namespace modbus {
void initialize();
Modbus slave(MODBUS_SLAVE_ID, PIN_RS485_MODBUS_TE);
/* 0x03 Read Multiple Holding Registers */
uint8_t read_holding_registers(
  uint8_t fc,
  uint16_t startaddress,
  uint16_t length);
/* 0x04 Read Input Registers */
uint8_t read_input_registers(
  uint8_t fc,
  uint16_t startaddress,
  uint16_t length);
/* 0x06 Write Single and 0x10 Write Multiple Holding Registers */
uint8_t write_holding_registers(
  uint8_t fc,
  uint16_t startaddress,
  uint16_t length);
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
namespace holding {
const uint8_t Pid = 6;
const uint8_t Manual = 1;
}
namespace input {
const uint8_t Output = 1;
const uint8_t Sensor = 1;
}
}
namespace index {
namespace holding {
namespace pid {
const uint8_t Setpoint = 0;
const uint8_t Kp = 1;
const uint8_t Ki = 2;
const uint8_t Kd = 3;
const uint8_t Ti = 4;
const uint8_t Td = 5;
}
const uint8_t Manual = 0;
}
namespace input {
const uint8_t Output = 0;
const uint8_t Temperature = 0;
}
}

namespace holding {
gatl::binding::reference<type::Output, uint16_t> manual;
gatl::binding::reference<type::Real, uint16_t> pid;
}
namespace input {
gatl::binding::reference<type::Output, uint16_t> output;
gatl::binding::reference<::gos::meltingpoint::type::Real, uint16_t> sensor;
}
void create();
} /* End of modbus binding name-space */
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
namespace set {
gatl::buffer::Holder<> kp(TEXT_ID_SET_KP, sizeof(TEXT_ID_SET_KP));
gatl::buffer::Holder<> ki(TEXT_ID_SET_KI, sizeof(TEXT_ID_SET_KI));
gatl::buffer::Holder<> kd(TEXT_ID_SET_KD, sizeof(TEXT_ID_SET_KD));
gatl::buffer::Holder<> ti(TEXT_ID_SET_TI, sizeof(TEXT_ID_SET_TI));
gatl::buffer::Holder<> td(TEXT_ID_SET_TD, sizeof(TEXT_ID_SET_TD));
}
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
Tick interfaces(INTERVAL_INTERFACE);
} /* End of timer name-space */

namespace push {
namespace button {
uint16_t value;
enum class status { idle, momentarily, prolonged };
status state(const unsigned long& tick, const uint16_t& value);
status last;
}
} /* End of push button name-space */

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

namespace pid {
::gos::atl::pid::Parameter<
  ::gos::meltingpoint::type::Real,
  ::gos::meltingpoint::type::Output,
  ::gos::meltingpoint::type::Real> parameter;
::gos::atl::pid::Variable<::gos::meltingpoint::type::Real> variable;
namespace tune {
::gos::atl::pid::Tune<::gos::meltingpoint::type::Real> k;
::gos::atl::pid::TimeTune<::gos::meltingpoint::type::Real> t;
namespace process {
void kp();
void ki();
void kd();
void ti();
void td();
}
}
} /* End of pid name-space */

namespace potentiometer {
typedef uint16_t Type;
typedef gatl::type::optional<Type> Optional;
Type value;
namespace change {
namespace sensitivity {
const Type ForSetpoint = 4;
const Type ForManual = 4;
}
}
namespace to {
const type::Real setpoint(const Type& value);
const type::Real tune(const Type& value);
const type::Output manual(const Type& value);
}
namespace last {
Optional setpoint;
Optional manual;
}
void process(const int& value);
} /* End of potentiometer name-space */

namespace heater {
typedef uint8_t Type;
Type value = 0;
type::Output manual = value::zero::Output;
bool apply(const type::Output& value);
bool apply();
} /* End of heater name-space */

namespace eeprom {
namespace binding {
namespace index {
namespace pid {
const uint8_t Kp = 0;
const uint8_t Ki = 1;
const uint8_t Kd = 2;
const uint8_t Ti = 3;
const uint8_t Td = 4;
}
}
const uint8_t Count = 5;
gatl::binding::reference<::gos::meltingpoint::type::Real, int> pid;
void create();
void read();
}
} /* End of eeprom name-space */

}
}

namespace gm = ::gos::meltingpoint;
namespace gmp = ::gos::meltingpoint::pid;
namespace gmt = ::gos::meltingpoint::type;
namespace gme = ::gos::meltingpoint::eeprom;
namespace gmd = ::gos::meltingpoint::display;
namespace gmv = ::gos::meltingpoint::variables;
namespace gmpb = ::gos::meltingpoint::push::button;
namespace gmeb = ::gos::meltingpoint::eeprom::binding;
namespace gmvt = ::gos::meltingpoint::variables::temporary;

void setup() {
  gme::binding::create();
  gm::modbus::binding::create();
  gme::binding::read();

  pinMode(PIN_HEATER, OUTPUT);
  pinMode(PIN_PUSH_BUTTON, INPUT_PULLUP);

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
  gm::modbus::slave.begin(MODBUS_BAUD);
#else
  Serial.begin(SERIAL_BAUD);
#endif

  gm::pid::parameter.Range = gm::variables::range::output;
  gm::pid::parameter.TimeMs = INTERVAL_CYCLE;
  gatl::pid::tunings(
    gm::pid::variable,
    gm::pid::parameter,
    gm::pid::tune::k);

  /* Initial read */
  gm::sensor::read();
  gm::potentiometer::value = analogRead(PIN_POTENTIOMETER);
  gm::potentiometer::process(gm::potentiometer::value);
  gm::display::update::second::line();

#ifdef DELAY_SENSOR_SETUP_END
  delay(DELAY_SENSOR_SETUP_END);
#endif
}

void loop() {
  gm::variables::tick = millis();

  gm::push::button::value = analogRead(PIN_PUSH_BUTTON);
  gm::push::button::last = gm::push::button::state(gmv::tick, gmpb::value);
  switch (gm::push::button::last) {
  case gm::push::button::status::momentarily:
    if (gm::mode::is::setting) {
      switch (gm::mode::state) {
      case gm::mode::status::kp:
        gm::pid::tune::process::kp();
      case gm::mode::status::ki:
        gm::pid::tune::process::ki();
        break;
      case gm::mode::status::kd:
        gm::pid::tune::process::kd();
        break;
      case gm::mode::status::ti:
        gm::pid::tune::process::ti();
        break;
      case gm::mode::status::td:
        gm::pid::tune::process::td();
        break;
      }
      gm::mode::is::setting = false;
    }
    gm::mode::gotom::next();
    break;
  case gm::push::button::status::prolonged:
    gm::mode::is::setting = true;
    break;
  }

  if (gm::timer::interfaces.is(gmv::tick)) {
    gm::potentiometer::value = analogRead(PIN_POTENTIOMETER);
    gm::potentiometer::process(gm::potentiometer::value);
    gm::display::update::second::line();
    if (gm::mode::state == gm::mode::status::manual) {
#ifdef NOT_USED
      if (!gm::heater::apply()) {
        /* Error handling */
      }
#else
      gm::heater::apply();
#endif
    }
  }
  if (gm::timer::cycle.is(gmv::tick)) {
    gm::sensor::read();
    if (gm::mode::state == gm::mode::status::automatic) {
      gm::variables::output = gatl::pid::compute<gm::type::Real, gm::type::Output>(
        gmv::temperature, gm::pid::variable, gm::pid::parameter);
#ifdef NOT_USED
      if (!gm::heater::apply(gm::variables::output)) {
        /* Error handling */
      }
#else
      gm::heater::apply(gm::variables::output);
#endif
    }
#ifndef MODBUS_BAUD
    Serial.print(gmv::temperature);
    Serial.print(",");
    Serial.print(gm::pid::parameter.Setpoint);
    Serial.print(",");
    Serial.print(gm::heater::value);
    Serial.println();
#endif
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
namespace meltingpoint {
namespace mode {
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
  if (state == gm::mode::status::automatic) {
    gm::variables::output =
      static_cast<gm::type::Output>(gm::heater::manual);
    gatl::pid::initialize<gm::type::Real, gm::type::Real, gm::type::Output>(
      gm::pid::variable,
      gm::pid::parameter.Range,
      gmv::temperature,
      gmv::output);
  }
}

}
}

namespace heater {
bool apply() {
  return apply(gm::heater::manual);
}
bool apply(const type::Output& value) {
  if (gatl::utility::range::isinside<gm::type::Output>(
    value,
    gm::variables::range::output)) {
    gm::heater::value = static_cast<gm::heater::Type>(value);
    analogWrite(PIN_HEATER, gm::heater::value);
    return true;
  } else {
    return false;
  }
}
} /* End of heater name-space */


namespace pid {
namespace tune {
namespace process {
void kp() {
  gatl::eeprom::update(gmeb::pid, gmeb::index::pid::Kp);
  gatl::pid::tunings(
    gm::pid::variable,
    gm::pid::parameter,
    gm::pid::tune::k);
}
void ki() {
  gm::pid::tune::t.Ti = gatl::pid::Ti(
    gm::pid::parameter.Kp, gm::pid::tune::k.Ki);
  gatl::eeprom::update(gmeb::pid, gmeb::index::pid::Ti);
  gatl::eeprom::update(gmeb::pid, gmeb::index::pid::Ki);
  gatl::pid::tunings(
    gm::pid::variable,
    gm::pid::parameter,
    gm::pid::tune::k);
}
void kd() {
  gm::pid::tune::t.Td = gatl::pid::Td(
    gm::pid::parameter.Kp, gm::pid::tune::k.Kd);
  gatl::eeprom::update(gmeb::pid, gmeb::index::pid::Td);
  gatl::eeprom::update(gmeb::pid, gmeb::index::pid::Kd);
  gatl::pid::tunings(
    gm::pid::variable,
    gm::pid::parameter,
    gm::pid::tune::k);
}
void ti() {
  gm::pid::tune::k.Ki = gatl::pid::Ki(
    gm::pid::parameter.Kp, gm::pid::tune::t.Ti);
  gatl::eeprom::update(gmeb::pid, gmeb::index::pid::Ti);
  gatl::eeprom::update(gmeb::pid, gmeb::index::pid::Ki);
  gatl::pid::tunings(
    gm::pid::variable,
    gm::pid::parameter,
    gm::pid::tune::t);
}
void td() {
  gm::pid::tune::k.Kd = gatl::pid::Kd(
    gm::pid::parameter.Kp, gm::pid::tune::t.Td);
  gatl::eeprom::update(gmeb::pid, gmeb::index::pid::Td);
  gatl::eeprom::update(gmeb::pid, gmeb::index::pid::Kd);
  gatl::pid::tunings(
    gm::pid::variable,
    gm::pid::parameter,
    gm::pid::tune::t);
}
}
}
}

namespace display {
namespace update {
namespace second {
void line() {
  switch (gm::mode::state) {
  case gm::mode::status::idle:
    gatl::format::real(
      gm::format::display::buffer::second ,
      gm::pid::parameter.Setpoint,
      gm::format::display::option::temperature,
      &gm::format::display::buffer::id::idle,
      &gm::format::display::buffer::unit);
    break;
  case gm::mode::status::manual:
    gatl::format::integer(
      gm::format::display::buffer::second,
      gm::heater::manual,
      &gm::format::display::buffer::id::manual);
    break;
  case gm::mode::status::automatic:
    gatl::format::real(
      gm::format::display::buffer::second,
      gm::pid::parameter.Setpoint,
      gm::format::display::option::temperature,
      &gm::format::display::buffer::id::setpoint,
      &gm::format::display::buffer::unit);
    break;
  case gm::mode::status::kp:
    gatl::format::real(
      gm::format::display::buffer::second,
      gm::pid::parameter.Kp,
      gm::format::display::option::tuning,
      gm::mode::is::setting ?
      &gm::format::display::buffer::id::tune::set::kp :
      &gm::format::display::buffer::id::tune::kp);
    break;
  case gm::mode::status::ki:
    gatl::format::real(
      gm::format::display::buffer::second,
      gm::pid::tune::k.Ki,
      gm::format::display::option::tuning,
      gm::mode::is::setting ?
      &gm::format::display::buffer::id::tune::set::ki :
      &gm::format::display::buffer::id::tune::ki);
    break;
  case gm::mode::status::kd:
    gatl::format::real(
      gm::format::display::buffer::second,
      gm::pid::tune::k.Kd,
      gm::format::display::option::tuning,
      gm::mode::is::setting ?
      &gm::format::display::buffer::id::tune::set::kd :
      &gm::format::display::buffer::id::tune::kd);
    break;
  case gm::mode::status::ti:
    gatl::format::real(
      gm::format::display::buffer::second,
      gm::pid::tune::t.Ti,
      gm::format::display::option::tuning,
      gm::mode::is::setting ?
      &gm::format::display::buffer::id::tune::set::ti :
      &gm::format::display::buffer::id::tune::ti);
    break;
  case gm::mode::status::td:
    gatl::format::real(
      gm::format::display::buffer::second,
      gm::pid::tune::t.Td,
      gm::format::display::option::tuning,
      gm::mode::is::setting ?
      &gm::format::display::buffer::id::tune::set::td :
      &gm::format::display::buffer::id::tune::td);
    break;
  }
  gm::display::updated = true;
}
}
}
}

namespace potentiometer {
namespace to {
const gm::type::Real setpoint(const int& value) {
  return static_cast<gm::type::Real>(map(value,
    POTENTIOMETER_RAW_MININUM, POTENTIOMETER_RAW_MAXIMUM,
    SETPOINT_MININUM, SETPOINT_MAXIMUM));
}
const gm::type::Real tune(const int& value) {
  return static_cast<gm::type::Real>(value) / 64.0F;
}
const type::Output manual(const int& value) {
  return static_cast<type::Output>(map(value,
    POTENTIOMETER_RAW_MININUM, POTENTIOMETER_RAW_MAXIMUM,
    PID_MINIMUM_OUTPUT, PID_MAXIMUM_OUTPUT));
}
}
namespace details {
namespace converted {
type::Output manual;
}
}
void process(const int& value) {
  switch (gm::mode::state) {
  case gm::mode::status::idle:
  case gm::mode::status::automatic:
    if (gatlu::changed::apply::is<Type>(
      value,
      last::setpoint,
      change::sensitivity::ForSetpoint)) {      
      gm::pid::parameter.Setpoint = to::setpoint(value);
      gm::modbus::variables::setpoint = gm::pid::parameter.Setpoint;
      gm::modbus::variables::last::setpoint = gm::pid::parameter.Setpoint;
    }
    break;
  case gm::mode::status::manual:
    if (gatlu::changed::apply::is<Type>(
      value,
      last::manual,
      change::sensitivity::ForManual)) {
      gm::heater::manual = to::manual(value);
      gm::modbus::variables::manual = gm::heater::manual;
      gm::modbus::variables::last::manual = gm::heater::manual;
    }
    break;
  case gm::mode::status::kp:
    if (gm::mode::is::setting) {
      gm::pid::parameter.Kp = to::tune(value);
    }
    break;
  case gm::mode::status::ki:
    if (gm::mode::is::setting) {
      gm::pid::tune::k.Ki = to::tune(value);
    }
    break;
  case gm::mode::status::kd:
    if (gm::mode::is::setting) {
      gm::pid::tune::k.Kd = to::tune(value);
    }
    break;
  case gm::mode::status::ti:
    if (gm::mode::is::setting) {
      gm::pid::tune::t.Ti = to::tune(value);
    }
    break;
  case gm::mode::status::td:
    if (gm::mode::is::setting) {
      gm::pid::tune::t.Td = to::tune(value);
    }
    break;
  }
}
}


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
}

namespace push {
namespace button {
namespace details {
namespace active {
bool is = false;
bool islong = false;
unsigned long time;
}
unsigned long rest = 0;
}
status state(const unsigned long& tick, const uint16_t& value) {
  status result = status::idle;
  if (push::button::details::rest == 0) {
    if (value < PUSH_BUTTON_ANALOG_THREASHOLD) {
      if (!push::button::details::active::is) {
        push::button::details::active::is = true;
        push::button::details::active::time = tick;
      }
      if ((tick - push::button::details::active::time) >
        PUSH_BUTTON_LONG_PRESS_TIME && !push::button::details::active::islong) {
        push::button::details::active::islong = true;
        result = status::prolonged;
        push::button::details::rest = tick;
      }
    } else {
      if (push::button::details::active::is) {
        if (push::button::details::active::islong) {
          push::button::details::active::islong = false;
        } else {
          result = status::momentarily;
          push::button::details::rest = tick;
        }
        push::button::details::active::is = false;
      }
    }
  } else {
    if ((tick - push::button::details::rest) > PUSH_BUTTON_REST_TIME) {
      push::button::details::rest = 0;
    }
  }
  return result;
}
}
}

namespace eeprom {
namespace binding {
void create() {
  gmv::temporary::address = gatl::binding::create<gm::type::Real, int, uint8_t>(
    gme::binding::pid,
    0,
    Count,
    sizeof(gm::type::Real));
  gatl::binding::set<gm::type::Real, int, uint8_t>(
    gme::binding::pid,
    index::pid::Kp,
    &gm::pid::parameter.Kp);
  gatl::binding::set<gm::type::Real, int, uint8_t>(
    gme::binding::pid,
    index::pid::Ki,
    &gm::pid::tune::k.Ki);
  gatl::binding::set<gm::type::Real, int, uint8_t>(
    gme::binding::pid,
    index::pid::Kd,
    &gm::pid::tune::k.Kd);
  gatl::binding::set<gm::type::Real, int, uint8_t>(
    gme::binding::pid,
    index::pid::Ti,
    &gm::pid::tune::t.Ti);
  gatl::binding::set<gm::type::Real, int, uint8_t>(
    gme::binding::pid,
    index::pid::Td,
    &gm::pid::tune::t.Td);
}
void read() {
  gatl::eeprom::read(gm::eeprom::binding::pid);
}
}
}

namespace modbus {
/* 0x03 Read Multiple Holding Registers */
uint8_t read_holding_registers(
  uint8_t fc,
  uint16_t startaddress,
  uint16_t length) {
  gmvt::status = STATUS_ILLEGAL_DATA_ADDRESS;
  if (gatl::modbus::two::access(
    binding::holding::pid,
    slave,
    startaddress,
    length)) {
    gmvt::status = STATUS_OK;
  }
  return gmvt::status;
}
/* 0x04 Read Input Registers */
uint8_t read_input_registers(
  uint8_t fc,
  uint16_t startaddress,
  uint16_t length) {
  gmvt::status = STATUS_ILLEGAL_DATA_ADDRESS;
  if (gatl::modbus::registers::access(
    binding::input::output,
    slave,
    startaddress,
    length)) {
    gmvt::status = STATUS_OK;
  }
  if (gatl::modbus::two::access(
    binding::input::sensor,
    slave,
    startaddress,
    length)) {
    gmvt::status = STATUS_OK;
  }
  return gmvt::status;
}
/* 0x06 Write Single and 0x10 Write Multiple Holding Registers */
uint8_t write_holding_registers(
  uint8_t fc,
  uint16_t startaddress,
  uint16_t length) {
  gmvt::status = STATUS_ILLEGAL_DATA_ADDRESS;
  if (gatl::modbus::registers::assign(
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
  if (gatl::modbus::two::assign(
    binding::holding::pid,
    slave,
    startaddress,
    length,
    gmvt::index,
    gmvt::to)) {
    gmvt::status = STATUS_OK;
    while (gmvt::index < gmvt::to) {
      if (gmvt::index == binding::index::holding::pid::Setpoint) {
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
        /* No need to do anything */
      } else if (gmvt::index == binding::index::holding::pid::Kp) {
        gm::pid::tune::process::kp();
      } else if (gmvt::index == binding::index::holding::pid::Ki) {
        gm::pid::tune::process::ki();
      } else if (gmvt::index == binding::index::holding::pid::Kd) {
        gm::pid::tune::process::kd();
      } else if (gmvt::index == binding::index::holding::pid::Ti) {
        gm::pid::tune::process::ti();
      } else if (gmvt::index == binding::index::holding::pid::Td) {
        gm::pid::tune::process::td();
      }
      gmvt::index++;
    }
  }
  return gmvt::status;
}
void initialize() {
#ifdef NOT_USED
  slave.cbVector[CB_READ_COILS] = read_coils;
  slave.cbVector[CB_READ_DISCRETE_INPUTS] = read_discrete_inputs;
#endif
  slave.cbVector[CB_READ_INPUT_REGISTERS] = read_input_registers;
  slave.cbVector[CB_READ_HOLDING_REGISTERS] = read_holding_registers;
#ifdef NOT_USED
  slave.cbVector[CB_WRITE_COILS] = write_coils;
#endif
  slave.cbVector[CB_WRITE_HOLDING_REGISTERS] = write_holding_registers;
}
namespace binding {
void create() {
  gmvt::address = gatl::binding::create<gm::type::Output, uint16_t, uint8_t>(
    binding::holding::manual,
    0,
    binding::count::holding::Manual,
    sizeof(gm::type::Output) / 2);
  gatl::binding::set<gm::type::Output, uint16_t, uint8_t>(
    binding::holding::manual,
    index::holding::Manual,
    &variables::manual);

  gmvt::address = gatl::binding::create<gm::type::Real, uint16_t, uint8_t>(
    binding::holding::pid,
    gmvt::address,
    binding::count::holding::Pid,
    sizeof(gm::type::Real) / 2);
  gatl::binding::set< gm::type::Real, uint16_t, uint8_t>(
    binding::holding::pid,
    index::holding::pid::Setpoint,
    &variables::setpoint);
  gatl::binding::set<gm::type::Real, uint16_t, uint8_t>(
    binding::holding::pid,
    index::holding::pid::Kp,
    &gm::pid::parameter.Kp);
  gatl::binding::set<gm::type::Real, uint16_t, uint8_t>(
    binding::holding::pid,
    index::holding::pid::Ki,
    &gm::pid::tune::k.Ki);
  gatl::binding::set<gm::type::Real, uint16_t, uint8_t>(
    binding::holding::pid,
    index::holding::pid::Kd,
    &gm::pid::tune::k.Kd);
  gatl::binding::set<gm::type::Real, uint16_t, uint8_t>(
    binding::holding::pid,
    index::holding::pid::Ti,
    &gm::pid::tune::t.Ti);
  gatl::binding::set<gm::type::Real, uint16_t, uint8_t>(
    binding::holding::pid,
    index::holding::pid::Td,
    &gm::pid::tune::t.Td);
  
  gmvt::address = gatl::binding::create<gm::type::Output, uint16_t, uint8_t>(
    binding::input::output,
    0,
    binding::count::input::Output,
    sizeof(gm::type::Output));
  gatl::binding::set<gm::type::Output, uint16_t, uint8_t>(
    binding::input::output,
    index::input::Output,
    &gm::variables::output);
  
  gmvt::address = gatl::binding::create<gm::type::Real, uint16_t, uint8_t>(
    binding::input::sensor,
    gmvt::address,
    binding::count::input::Sensor,
    sizeof(gm::type::Real));
  gatl::binding::set<gm::type::Real, uint16_t, uint8_t>(
    binding::input::sensor,
    index::input::Temperature,
    &gm::variables::temperature);

}
}
}

}
}
