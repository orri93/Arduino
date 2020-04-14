#include "sensor.h"

/*
 * SPI
 *                      UNO
 * 1 SLK <  SCK      Pin 13
 * 2 CS  <  D10      Pin 10
 * 3 DO  > MISO      Pin 12
 * 4 VCC    VCC      3.3/5V
 * 5 GND    GND         GND
 */

#include <gatlled.h>
#include <gatltick.h>
#include <gatlmodbus.h>

#include "pid.h"
#include "macro.h"
#include "eeprom.h"
#include "modbus.h"
#include "format.h"
#include "display.h"
#include "variable.h"
#include "fds-celsius-logo.h"

namespace gatl = ::gos::atl;

namespace gt = ::gos::temperature;
namespace gtv = ::gos::temperature::variables;
namespace gtvi = ::gos::temperature::variables::timing;
namespace gtfdb = ::gos::temperature::format::display::buffer;

typedef gatl::Tick<GATL_TICK_DEFAULT_TYPE, GATL_TICK_DEFAULT_TYPE> Tick;

Tick displaytick(DISPLAY_INTERVAL);

void setup() {
#ifndef NO_DISPLAY
  gt::display::oled.U8g2->begin();
#ifndef NO_DISPLAY_LOGO
  gatl::display::synchronous::logo(
    gt::display::oled,
    fds_celsius_logo_width,
    fds_celsius_logo_height,
    fds_celsius_logo_bits);
#endif
#endif

  gt::eeprom::retrieve::initial();

  gt::format::initialize();

#ifdef SUPPORT_PONE
  gt::pid::parameter.PonE = bitRead(gtv::modbus::coils, GOS_TCV_COIL_BIT_PONE);
#endif
  gt::pid::create();
  gt::pid::tune::time();

  gt::sensor::temperature.begin();

  /* Serial for RS485 */
  Serial.begin(MODBUS_BAUD);

  gt::modbus::initialize();

  pinMode(PIN_HEATER, OUTPUT);

  gatl::led::initialize(PIN_LED_RED);
  gatl::led::initialize(PIN_LED_BLUE);
  gatl::led::initialize(PIN_LED_GREEN);
  gatl::led::initialize(PIN_LED_YELLOW);

  gatl::led::blink(PIN_LED_MODBUS_READ);
  gatl::led::blink(PIN_LED_MODBUS_WRITE);
  gatl::led::blink(PIN_LED_RED);
  gatl::led::blink(PIN_LED_BLUE);
}

void loop() {
  gtvi::tick = millis();

  gatl::modbus::loop<MODBUS_TYPE_DEFAULT>(
    Serial,
    gt::modbus::parameter,
    gt::modbus::handler,
    gt::modbus::variable,
    gt::modbus::buffer::request,
    gt::modbus::buffer::response);

  if (gatl::tick::is::next<GATL_TICK_DEFAULT_TYPE, gt::type::Unsigned>(
      gtvi::next, gtvi::tick, gtvi::interval)) {
    switch (gtv::force) {
    case GOT_PI_FORCE_IDLE:
      gtv::status = gt::type::Status::idle;
      break;
    case GOT_PI_FORCE_MANUAL:
      gtv::status = gt::type::Status::manual;
      break;
    case GOT_PI_FORCE_AUTO:
      gtv::status = gt::type::Status::automatic;
      break;
    }
    if(gtv::status == gt::type::Status::automatic) {
      digitalWrite(PIN_LED_BLUE, HIGH);
    }
    gt::sensor::read();
    switch (gtv::status) {
    case gt::type::Status::manual:
      gatl::pid::wiki::initialize(gt::pid::variable);
      gtv::output = gtv::controller::manual;
      break;
    case gt::type::Status::automatic:
      switch(gt::sensor::temperature.Last) {
      case gatl::sensor::Status::Operational:
      case gatl::sensor::Status::BelowRange:
      case gatl::sensor::Status::AboveRange:
        gtv::output = static_cast<gt::type::Unsigned>(gatl::pid::wiki::compute<
          gt::type::Real,
          gt::type::Real,
          gt::type::Real,
          gt::type::Real>(
            gtv::temperature,
            gt::pid::variable,
            gt::pid::parameter,
            gt::pid::tune::k));
        break;
      default:
        gatl::pid::wiki::initialize(gt::pid::variable);
        gtv::output = 0;
        break;
      }
      break;
    case gt::type::Status::idle:
    default:
      gatl::pid::wiki::initialize(gt::pid::variable);
      gtv::output = 0;
      break;
    }
    analogWrite(PIN_HEATER, gtv::output);

   if(gtv::status == gt::type::Status::automatic) {
      digitalWrite(PIN_LED_BLUE, LOW);
   }
  }

#ifndef NO_DISPLAY
  if (displaytick.is(gtvi::tick)) {
    gt::sensor::report();
    gt::display::two.display(gtfdb::first, gtfdb::second);
  }
  gt::display::two.loop();
#endif
}
