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

#ifndef NO_DISPLAY
#include <gatldisplay.h>
#include <gatlformat.h>

namespace gatl = ::gos::atl;

#define TEXT_ID_IMMEDIATE "T: "
#define TEXT_ID_SETPOINT "S: "

#define TEXT_UNIT " C"

namespace gos {
namespace meltingpoint {

namespace format {
namespace display {
gatl::format::option::Number option;
namespace buffer {
gatl::buffer::Holder<> immediate;
gatl::buffer::Holder<> setpoint;
namespace id {
gatl::buffer::Holder<> immediate(TEXT_ID_IMMEDIATE, sizeof(TEXT_ID_IMMEDIATE));
gatl::buffer::Holder<> setpoint(TEXT_ID_SETPOINT, sizeof(TEXT_ID_SETPOINT));
}
gatl::buffer::Holder<> unit(TEXT_UNIT, sizeof(TEXT_UNIT));
}
}
}
namespace display {
gatl::display::Oled<> oled;
gatl::display::asynchronous::line::Two<> twoline(oled);
}
#endif

#define PIN_MAX6675_CS           8

#define PIN_PUSH_BUTTON         A0

#define PIN_POTENTIOMETER       A2
#define PIN_HEATER               6

#define INTERVAL_CYCLE        1000

#define DELAY_SENSOR_SETUP_END 500

#define SERIAL_BAUD          19200

#define SENSOR_MINIMUM                       0
#define SENSOR_MAXIMUM                     500

#define POTENTIOMETER_SETPOINT_MININUM       0
#define POTENTIOMETER_SETPOINT_MAXIMUM     300

#define PID_MINIMUM_OUTPUT                   0
#define PID_MAXIMUM_OUTPUT                 255

Tick timer(INTERVAL_CYCLE);

namespace push {
namespace button {
gatl::statistics::Set<double> set(NAN);
gatl::statistics::Median<double> median(set);
}
}

::gos::Max6675 max6675(PIN_MAX6675_CS);

namespace pid {
namespace type {
typedef float Input;
typedef uint8_t Output;
typedef float Parameter;
typedef float Variable;
typedef float Tune;
}
::gos::atl::pid::Parameter<
  ::gos::meltingpoint::pid::type::Input,
  ::gos::meltingpoint::pid::type::Output,
  ::gos::meltingpoint::pid::type::Parameter> parameter;
::gos::atl::pid::Tune<::gos::meltingpoint::pid::type::Tune> tune;
::gos::atl::pid::Variable<::gos::meltingpoint::pid::type::Variable> variable;
}

namespace eeprom {
int address;
namespace binding {
gatl::binding::reference<::gos::meltingpoint::pid::type::Parameter, int> parameter;
gatl::binding::reference<::gos::meltingpoint::pid::type::Tune, int> tune;
}
}

unsigned long tick;

const char* error6675;
uint8_t status6675, length6675;
double value6675, median6675, temperature;

uint16_t push_button, potentiometer, heater;

}
}

namespace gm = ::gos::meltingpoint;
namespace gme = ::gos::meltingpoint::eeprom;

void setup() {
  gme::address = gatl::binding::create<gm::pid::type::Parameter, int, uint8_t>(
    ::gos::meltingpoint::eeprom::binding::parameter,
    0,
    1,
    sizeof(gm::pid::type::Parameter));
  gatl::binding::set<gm::pid::type::Parameter, int, uint8_t>(
    ::gos::meltingpoint::eeprom::binding::parameter,
    0,
    &gm::pid::parameter.Kp);
  gme::address = gatl::binding::create<gm::pid::type::Tune, int, uint8_t>(
    ::gos::meltingpoint::eeprom::binding::tune,
    gme::address,
    2,
    sizeof(gm::pid::type::Tune));
  gatl::binding::set<gm::pid::type::Tune, int, uint8_t>(
    ::gos::meltingpoint::eeprom::binding::tune,
    0,
    &gm::pid::tune.Ki);
  gatl::binding::set<gm::pid::type::Tune, int, uint8_t>(
    ::gos::meltingpoint::eeprom::binding::tune,
    1,
    &gm::pid::tune.Kd);

  pinMode(PIN_HEATER, OUTPUT);

#ifndef NO_DISPLAY
  ::gos::meltingpoint::display::oled.U8g2->begin();
#ifdef USE_MEDIAN
  displaybuffermedian.Buffer[0] = '\0';
#endif
#endif

  SPI.begin();

  ::gos::meltingpoint::max6675.initialize();

#ifdef SERIAL_BAUD
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
    delay(10);
  }
  Serial.println("MAX 6675 testing");
#endif

::gos::meltingpoint::pid::parameter.Range =
  ::gos::atl::type::make_range<::gos::meltingpoint::pid::type::Output>(
  PID_MINIMUM_OUTPUT,
  PID_MAXIMUM_OUTPUT);
::gos::meltingpoint::pid::parameter.TimeMs = INTERVAL_CYCLE;

#ifdef DELAY_SENSOR_SETUP_END
  delay(DELAY_SENSOR_SETUP_END);
#endif
}

void loop() {
  ::gos::meltingpoint::tick = millis();

  gm::push_button = analogRead(PIN_PUSH_BUTTON);

  if (gm::timer.is(gm::tick)) {

  gm::potentiometer = analogRead(PIN_POTENTIOMETER);
  gm::error6675 = nullptr;
  if (gm::max6675.read(gm::value6675)) {
    if (::gos::sensor::range::check(
      gm::value6675,
      SENSOR_MINIMUM,
      SENSOR_MAXIMUM) == GOS_SENSOR_STATUS_OK) {
#ifdef USE_MEDIAN
      set.add(value6675);
#endif
    } else {
      gm::error6675 = ::gos::sensor::error(gm::length6675);
    }
  } else {
    gm::error6675 = gm::max6675.error(gm::length6675);
  }
    Serial.print(gm::potentiometer);
    Serial.print(",");
    if (gm::error6675 == nullptr) {
      Serial.print(gm::value6675, 1);
#ifdef USE_MEDIAN
      if (set.Count > 6) {
        median6675 = median.get();
        Serial.print(",");
        Serial.println(median6675, 1);
      }
#endif
      Serial.println();
#ifndef NO_DISPLAY
#ifdef USE_MEDIAN
      temperature = set.Count > 6 ? median6675 : value6675;
#else
      gm::temperature = gm::value6675;
#endif
      ::gos::atl::format::real(
        gm::format::display::buffer::immediate,
        gm::temperature,
        gm::format::display::option,
        &gm::format::display::buffer::id::immediate,
        &gm::format::display::buffer::unit);
#endif
    } else {
      Serial.println(gm::error6675);
#ifndef NO_DISPLAY
      gatl::format::message(
        gm::format::display::buffer::immediate,
        gm::error6675,
        gm::length6675,
        &gm::format::display::buffer::id::immediate);
#endif
    }
    gatl::format::real(
      gm::format::display::buffer::setpoint,
      gm::pid::parameter.Setpoint,
      gm::format::display::option,
      &gm::format::display::buffer::id::setpoint,
      &gm::format::display::buffer::unit);
#ifndef NO_DISPLAY
    gm::display::twoline.display(
      gm::format::display::buffer::immediate,
      gm::format::display::buffer::setpoint);
#endif
  }

#ifndef NO_DISPLAY
  gm::display::twoline.loop();
#endif
}
