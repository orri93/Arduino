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
  gatl::string::copy(gm::format::display::buffer::first, TEXT_INITIAL_1);
  gatl::buffer::clear(gm::format::display::buffer::second);
  gmd::two.display(
    gm::format::display::buffer::first,
    gm::format::display::buffer::second);
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

#ifndef NO_DISPLAY
  if (firsttime) {
    gatl::string::copy(gm::format::display::buffer::first, TEXT_INITIAL_2);
    gmd::two.display(
      gm::format::display::buffer::first,
      gm::format::display::buffer::second);
    firsttime = false;
  }

  gm::display::two.loop();
#endif
}
