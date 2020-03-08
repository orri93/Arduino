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
#include <gatlmodbus.h>

#include "macro.h"
#include "eeprom.h"
#include "modbus.h"
#include "binding.h"
#include "display.h"
#include "fds-celsius-logo.h"

namespace gatl = ::gos::atl;

namespace gt = ::gos::temperature;

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

  gt::binding::create();

  gt::eeprom::retrieve::initial();

  /* Serial for RS485 */
  Serial.begin(MODBUS_BAUD);

  gt::modbus::initialize();

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
  gatl::modbus::loop<MODBUS_TYPE_DEFAULT>(
    Serial,
    gt::modbus::parameter,
    gt::modbus::handler,
    gt::modbus::variable,
    gt::modbus::buffer::request,
    gt::modbus::buffer::response);
}
