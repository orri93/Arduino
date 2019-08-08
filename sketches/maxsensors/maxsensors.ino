#include "define.h"

#include <gosmax31865.h>
#include <arduinosensor.h>
#include <arduinotick.h>

#ifndef NO_DISPLAY
#include <arduinoformat.h>
#include <arduinodisplay.h>
#endif

#include "pin.h"

#define INTERVAL_MAX_31865        500

#define DELAY_SENSOR_SETUP_END    500

#define SERIAL_BAUD            115200

/* For PT100 set type to 1 and for PT1000 set type to 2 */
#define MAX_RTD_TYPE  RTD_TYPE_PT1000

// set to 2WIRE or 4WIRE as necessary
#define MAX_WIRES          RTD_3_WIRE

namespace gs = ::gos::sensor;

Tick timermax_31865(INTERVAL_MAX_31865);

::gos::Max31865 max31865(PIN_MAX_31865_CS);

unsigned long tick;

const char* error_31865;
uint8_t length_31865;
double value_31865 = 0.0;

void setup() {
#ifndef NO_DISPLAY
  fds::format::setup();
  fds::format::set(
    ':',
    TEXT_UNIT,
    sizeof(TEXT_UNIT),
    DISPLAY_LENGTH,
    DISPLAY_LENGTH - sizeof(TEXT_UNIT) - 2,
    FORMAT_PRECISION);
  fds::format::ids(SENSOR_IDS, sizeof(SENSOR_IDS));
  fds::display::u8g2.begin();
#endif

  SPI.begin();

  max31865.initialize(MAX_RTD_TYPE, MAX_WIRES);

#ifdef SERIAL_BAUD
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
    delay(10);
  }
  Serial.println("MAX Sensors");
#endif

}

void loop() {
  tick = millis();

  if (timermax_31865.is(tick)) {
    error_31865 = nullptr;
    if (max31865.read(value_31865)) {
      if (gs::range::check(value_31865) != GOS_SENSOR_STATUS_OK) {
        error_31865 = gs::error(length_31865);
      }
    } else {
      error_31865 = max31865.error(length_31865);
    }
    if (error_31865 == nullptr) {
      Serial.println(value_31865, 1);
#ifndef NO_DISPLAY
      ::fds::format::number(value_31865, 0);
#endif
    }
    else {
      Serial.print(error_31865);
      Serial.print(" (");
      Serial.print(value_31865);
      Serial.println(")");
#ifndef NO_DISPLAY
      ::fds::format::error(error31865, length31865);
#endif
    }
  }
}
