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

#include <gosmax6675.h>
#include <arduinosensor.h>
#include <arduinotick.h>

#include <gatlbinding.h>
#include <gatleeprom.h>

#include <gatlpid.h>

/* Median for push button */
#include <gatlmedian.h>

#include <gatldisplay.h>
#include <gatlformat.h>

namespace gatl = ::gos::atl;

#define PIN_MAX6675_CS                       8
#define PIN_PUSH_BUTTON                     A0
#define PIN_POTENTIOMETER                   A2
#define PIN_HEATER                           6

#define INTERVAL_INTERFACE                 250
#define INTERVAL_CYCLE                    1000

#define DELAY_SENSOR_SETUP_END             500

#define SERIAL_BAUD                      19200

#define PUSH_BUTTON_ANALOG_THREASHOLD      512
#define PUSH_BUTTON_LONG_PRESS_TIME        250
#define PUSH_BUTTON_REST_TIME               25

#define SENSOR_MINIMUM                       0
#define SENSOR_MAXIMUM                     500

#define POTENTIOMETER_RAW_MININUM            0
#define POTENTIOMETER_RAW_MAXIMUM         1024

#define POTENTIOMETER_SETPOINT_MININUM       0
#define POTENTIOMETER_SETPOINT_MAXIMUM     300

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
status next(const status& state);
namespace is {
bool setting = false;
}
}

namespace type {
typedef float Real;
typedef uint8_t Output;
}

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
}
namespace display {
gatl::display::Oled<> oled;
gatl::display::asynchronous::line::Two<> twoline(oled);
bool updated = false;
namespace update {
namespace second {
void line();
}
}
}

namespace timer {
Tick cycle(INTERVAL_CYCLE);
Tick interfaces(INTERVAL_INTERFACE);
}

namespace push {
namespace button {
uint16_t value;
enum class status { idle, momentarily, prolonged };
status state(const unsigned long& tick, const uint16_t& value);
status last;
}
}

namespace sensor {
double value;
uint8_t status;
::gos::Max6675 max6675 (PIN_MAX6675_CS);
void read();
namespace error {
const char* message;
uint8_t length;
}
}

namespace pid {
::gos::atl::pid::Parameter<
  ::gos::meltingpoint::type::Real,
  ::gos::meltingpoint::type::Output,
  ::gos::meltingpoint::type::Real> parameter;
::gos::atl::pid::Variable<::gos::meltingpoint::type::Real> variable;
namespace tune {
::gos::atl::pid::Tune<::gos::meltingpoint::type::Real> k;
::gos::atl::pid::TimeTune<::gos::meltingpoint::type::Real> t;
}
}

namespace potentiometer {
uint16_t value;
namespace to {
const ::gos::meltingpoint::type::Real setpoint(const int& value);
const ::gos::meltingpoint::type::Real tune(const int& value);
const uint8_t manual(const int& value);
}
void process(const int& value);
}

namespace heater {
type::Output value;
}

namespace eeprom {
int address;
namespace binding {
gatl::binding::reference<::gos::meltingpoint::type::Real, int> pid;
}
}

unsigned long tick;

::gos::meltingpoint::type::Real temperature = 0.0F;

uint8_t manual;

}
}

namespace gm = ::gos::meltingpoint;
namespace gmp = ::gos::meltingpoint::pid;
namespace gmt = ::gos::meltingpoint::type;
namespace gme = ::gos::meltingpoint::eeprom;
namespace gmd = ::gos::meltingpoint::display;
namespace gmpb = ::gos::meltingpoint::push::button;

void setup() {
  gme::address = gatl::binding::create<gm::type::Real, int, uint8_t>(
    gme::binding::pid,
    0,
    5,
    sizeof(gm::type::Real));
  gatl::binding::set<gm::type::Real, int, uint8_t>(
    gme::binding::pid,
    0,
    &gm::pid::parameter.Kp);
  gatl::binding::set<gm::type::Real, int, uint8_t>(
    gme::binding::pid,
    1,
    &gm::pid::tune::k.Ki);
  gatl::binding::set<gm::type::Real, int, uint8_t>(
    gme::binding::pid,
    2,
    &gm::pid::tune::k.Kd);
  gatl::binding::set<gm::type::Real, int, uint8_t>(
    gme::binding::pid,
    3,
    &gm::pid::tune::t.Ti);
  gatl::binding::set<gm::type::Real, int, uint8_t>(
    gme::binding::pid,
    4,
    &gm::pid::tune::t.Td);

  gme::address = 0;
  gatl::eeprom::read(gm::eeprom::binding::pid);

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

#ifdef SERIAL_BAUD
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
    delay(10);
  }
  Serial.println("MAX 6675 testing");
#endif

  gmp::parameter.Range = gatl::type::make_range<gmt::Output>(
    PID_MINIMUM_OUTPUT,
    PID_MAXIMUM_OUTPUT);
  gmp::parameter.TimeMs = INTERVAL_CYCLE;

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
  ::gos::meltingpoint::tick = millis();

  gm::push::button::value = analogRead(PIN_PUSH_BUTTON);
  gm::push::button::last = gm::push::button::state(gm::tick, gmpb::value);
  switch (gm::push::button::last) {
  case gm::push::button::status::momentarily:
    if (gm::mode::is::setting) {
      switch (gm::mode::state) {
      case gm::mode::status::ki:
        gm::pid::tune::t.Ti = gatl::pid::Ti(
          gm::pid::parameter.Kp, gm::pid::tune::k.Ki);
        break;
      case gm::mode::status::kd:
        gm::pid::tune::t.Td = gatl::pid::Td(
          gm::pid::parameter.Kp, gm::pid::tune::k.Kd);
        break;
      case gm::mode::status::ti:
        gm::pid::tune::k.Ki = gatl::pid::Ki(
          gm::pid::parameter.Kp, gm::pid::tune::t.Ti);
        break;
      case gm::mode::status::td:
        gm::pid::tune::k.Kd = gatl::pid::Kd(
          gm::pid::parameter.Kp, gm::pid::tune::t.Td);
        break;
      }
      gatl::pid::tunings(
        gm::pid::variable,
        gm::pid::parameter,
        gm::pid::tune::k);
      gatl::eeprom::update(gm::eeprom::binding::pid);
      gm::mode::is::setting = false;
    }
    gm::mode::state = gm::mode::next(gm::mode::state);
    if (gm::mode::state == gm::mode::status::automatic) {
      gatl::pid::tunings(
        gm::pid::variable,
        gm::pid::parameter,
        gm::pid::tune::k);
      gatl::pid::initialize(
        gm::pid::variable, 
        gm::pid::parameter.Range,
        gm::temperature,
        gm::manual);
    }
    break;
  case gm::push::button::status::prolonged:
    gm::mode::is::setting = true;
    break;
  }

  if (gm::timer::interfaces.is(gm::tick)) {
    gm::potentiometer::value = analogRead(PIN_POTENTIOMETER);
    gm::potentiometer::process(gm::potentiometer::value);
    gm::display::update::second::line();
    if (gm::mode::state == gm::mode::status::manual) {
      analogWrite(PIN_HEATER, gm::manual);
    }
  }
  if (gm::timer::cycle.is(gm::tick)) {
    gm::sensor::read();
    if (gm::mode::state == gm::mode::status::automatic) {
      gm::heater::value = gatl::pid::compute<gm::type::Real, gm::type::Output>(
        gm::temperature, gm::pid::variable, gm::pid::parameter);
      analogWrite(PIN_HEATER, gm::heater::value);
    }
    Serial.print(gm::temperature);
    Serial.print(",");
    Serial.print(gm::pid::parameter.Setpoint);
    Serial.print(",");
    Serial.print(gm::heater::value);
    Serial.println();
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
      gm::manual,
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
    POTENTIOMETER_SETPOINT_MININUM, POTENTIOMETER_SETPOINT_MAXIMUM));
}
const gm::type::Real tune(const int& value) {
  return static_cast<gm::type::Real>(value) / 64.0F;
}
const uint8_t manual(const int& value) {
  return static_cast<uint8_t>(map(value,
    POTENTIOMETER_RAW_MININUM, POTENTIOMETER_RAW_MAXIMUM,
    PID_MINIMUM_OUTPUT, PID_MAXIMUM_OUTPUT));
}
}
void process(const int& value) {
  switch (gm::mode::state) {
  case gm::mode::status::idle:
  case gm::mode::status::automatic:
    gm::pid::parameter.Setpoint = to::setpoint(value);
    break;
  case gm::mode::status::manual:
    gm::manual = to::manual(value);
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
      gm::temperature = static_cast<gm::type::Real>(value);
    } else {
      error::message = ::gos::sensor::error(error::length);
    }
  } else {
    error::message = max6675.error(error::length);
  }
  if (gm::sensor::error::message == nullptr) {
    gatl::format::real(
      gm::format::display::buffer::first,
      gm::temperature,
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
}
}
