/*
 * SPI
 *                      UNO
 * 1 SLK <  SCK      Pin 13
 * 2 CS  <  D10      Pin 10
 * 3 DO  > MISO      Pin 12
 * 4 VCC    VCC      3.3/5V
 * 5 GND    GND         GND
 */

#include <EEPROM.h>

#include <gatlled.h>
#include <gatlstring.h>

#include "type.h"
#include "value.h"
#include "eeprom.h"
#include "variable.h"
#include "binding.h"
#include "macro.h"

#ifndef NO_DISPLAY
#include "format.h"
#include "display.h"
#endif

#include "modbus.h"

#define TEXT_INITIAL_1 "GOSMB1"
#define TEXT_INITIAL_2 "GOSMB2"

namespace gatl = ::gos::atl;
namespace gatll = ::gos::atl::led;

namespace gm = ::gos::modbus;
namespace gme = ::gos::modbus::eeprom;

#ifndef NO_DISPLAY
namespace gmd = ::gos::modbus::display;
#endif

bool firsttime = true;

void setup() {
  gm::binding::create();
  gme::binding::read();

  gatll::initialize(PIN_LED_RED);
  gatll::initialize(PIN_LED_BLUE);

  gatll::initialize(PIN_LED_GREEN);
  gatll::initialize(PIN_LED_YELLOW);

  /* RS485 */
  Serial.begin(MODBUS_BAUD);

  gm::initialize();

#ifndef NO_DISPLAY
  gmd::oled.U8g2->begin();
  gatl::string::copy(gm::format::display::buffer::first, TEXT_INITIAL_1);
  gatl::buffer::clear(gm::format::display::buffer::second);
  gmd::two.display(
    gm::format::display::buffer::first,
    gm::format::display::buffer::second);
#endif

  gm::variables::led::blue::last =
    gm::variables::led::blue::value = EEPROM.read(0);

  gatll::blink(PIN_LED_MODBUS_READ);
  gatll::blink(PIN_LED_MODBUS_WRITE);

  gatll::blink(PIN_LED_RED);

  gatll::blink(PIN_LED_BLUE);

  analogWrite(PIN_LED_BLUE, gm::variables::led::blue::value);
}

void loop() {
#ifdef USE_ARDUINO_MODBUS_SLAVE
  gm::slave.poll();
#else
#ifdef MODBUS_HANDLER_INTERFACE
  gatl::modbus::loop<uint16_t>(
    Serial,
    gm::parameter,
    gm::handler,
    gm::variable,
    gm::buffer::request,
    gm::buffer::response);
#else
  gatl::modbus::loop<uint16_t>(
    Serial,
    gm::parameter,
    gm::variable,
    gm::buffer::request,
    gm::buffer::response);
#endif
#endif

  gm::variables::led::red::value = bitRead(gm::variables::coils, 0);
  if (gm::variables::led::red::value != gm::variables::led::red::last) {
    digitalWrite(PIN_LED_RED, gm::variables::led::red::value ? HIGH : LOW);
    gm::variables::led::red::last = gm::variables::led::red::value;
  }

  if (gm::variables::led::blue::value != gm::variables::led::blue::last) {
    analogWrite(PIN_LED_BLUE, gm::variables::led::blue::value);
    gm::variables::led::blue::last = gm::variables::led::blue::value;
  }

  /*
  gm::variables::led::red::a = bitRead(gm::variables::coils, 0);
  gm::variables::led::red::b = bitRead(gm::variables::coils, 1);

  if (gm::variables::led::red::a != gm::variables::led::red::last::a) {
    digitalWrite(PIN_LED_RED_A, gm::variables::led::red::a ? HIGH : LOW);
    gm::variables::led::red::last::a = gm::variables::led::red::a;
  }
  if (gm::variables::led::red::b != gm::variables::led::red::last::b) {
    digitalWrite(PIN_LED_RED_B, gm::variables::led::red::b ? HIGH : LOW);
    gm::variables::led::red::last::b = gm::variables::led::red::b;
  }

  if (gm::variables::led::blue::a != gm::variables::led::blue::last::a) {
    analogWrite(PIN_LED_BLUE_A, gm::variables::led::blue::a);
    gm::variables::led::blue::last::a = gm::variables::led::blue::a;
  }
  if (gm::variables::led::blue::b != gm::variables::led::blue::last::b) {
    analogWrite(PIN_LED_BLUE_B, gm::variables::led::blue::b);
    gm::variables::led::blue::last::b = gm::variables::led::blue::b;
  }
  */

#ifndef NO_DISPLAY
  /*
  if (firsttime) {
    gatl::string::copy(gm::format::display::buffer::first, TEXT_INITIAL_2);
    gmd::two.display(
      gm::format::display::buffer::first,
      gm::format::display::buffer::second);
    firsttime = false;
  }
  */
  gm::display::two.loop();
#endif
}
