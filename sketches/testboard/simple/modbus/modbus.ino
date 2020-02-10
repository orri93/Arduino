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

#define TEXT_INITIAL "GOS MODBUS"

namespace gatl = ::gos::atl;
namespace gatll = ::gos::atl::led;

namespace gm = ::gos::modbus;
namespace gme = ::gos::modbus::eeprom;

#ifndef NO_DISPLAY
namespace gmd = ::gos::modbus::display;
#endif


void setup() {
  gm::binding::create();
  gme::binding::read();

  gatll::initialize(PIN_LED_RED_A);
  gatll::initialize(PIN_LED_RED_B);

  gatll::initialize(PIN_LED_BLUE_A);
  gatll::initialize(PIN_LED_BLUE_B);

  gatll::initialize(PIN_LED_GREEN);
  gatll::initialize(PIN_LED_YELLOW);

  pinMode(PIN_BUTTON_A, INPUT_PULLUP);
  pinMode(PIN_BUTTON_B, INPUT_PULLUP);

  gm::initialize();

  /* RS485 */
  Serial.begin(MODBUS_BAUD);

  gatl::modbus::begin<>(Serial, gm::parameter, gm::variable, MODBUS_BAUD);

#ifndef NO_DISPLAY
  gmd::oled.U8g2->begin();

  gatl::buffer::clear(gm::format::display::buffer::first);
  gatl::buffer::clear(gm::format::display::buffer::second);

  gm::format::display::crc::last::first =
    gatl::utility::crc::calculate<>(gm::format::display::buffer::first);
  gm::format::display::crc::last::second =
    gatl::utility::crc::calculate<>(gm::format::display::buffer::second);

  gm::display::update::first::line(TEXT_INITIAL);
#endif

  gatll::blink(PIN_LED_MODBUS_READ);
  gatll::blink(PIN_LED_MODBUS_WRITE);
}


void loop() {
  gatl::modbus::loop<uint16_t>(
    Serial,
    gm::parameter,
    gm::handler,
    gm::variable,
    gm::buffer::request,
    gm::buffer::response);

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
