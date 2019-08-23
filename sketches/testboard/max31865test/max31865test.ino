/* Build Options */
//#define NO_DISPLAY

#include <SPI.h>

#include <gosmax31865.h>
#include <arduinosensor.h>
#include <arduinotick.h>

#ifndef NO_DISPLAY
#include <gatldisplay.h>
#include <gatlformat.h>

#define TEXT_UNIT " C"

::gos::atl::format::option::Number displayoption;
::gos::atl::buffer::Holder<> displaybuffer;
::gos::atl::buffer::Holder<> displayunit(TEXT_UNIT, sizeof(TEXT_UNIT));
::gos::atl::display::Oled<> oled;
::gos::atl::display::line::One<> displayoneline(oled);
::gos::atl::buffer::Holder<>* nullholder = nullptr;
#endif

#define INTERVAL_MAX_31865        500

#define DELAY_SENSOR_SETUP_END    500

#define SERIAL_BAUD            115200

#define PIN_SPI_MAX_31865_CS        7

/* For PT100 set type to 1 and for PT1000 set type to 2 */
#define MAX_RTD_TYPE  RTD_TYPE_PT1000

// set to 2WIRE or 4WIRE as necessary
#define MAX_WIRES          RTD_3_WIRE

Tick timermax31865(INTERVAL_MAX_31865);

unsigned long tick;

const char* error31865;
uint8_t length31865;
double value31865;

::gos::Max31865 max31865(PIN_SPI_MAX_31865_CS);

void setup() {
#ifndef NO_DISPLAY
  oled.U8g2->begin();
#endif

  SPI.begin();

  max31865.initialize(MAX_RTD_TYPE, MAX_WIRES);

#ifdef SERIAL_BAUD
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
    delay(10);
  }
  Serial.println("MAX 31865 testing");
#endif

#ifdef DELAY_SENSOR_SETUP_END
  delay(DELAY_SENSOR_SETUP_END);
#endif
}

void loop() {
  tick = millis();

  if (timermax31865.is(tick)) {
    error31865 = nullptr;
    if (max31865.read(value31865)) {
      if (gos::sensor::range::check(value31865) != GOS_SENSOR_STATUS_OK) {
        error31865 = gos::sensor::error(length31865);
      }
    }
    else {
      error31865 = max31865.error(length31865);
    }
    if (error31865 == nullptr) {
      Serial.println(value31865, 1);
#ifndef NO_DISPLAY
      ::gos::atl::format::real(
        displaybuffer,
        value31865,
        displayoption,
        nullholder,
        &displayunit);
#endif
    }
    else {
      Serial.print(error31865);
      Serial.print(" (");
      Serial.print(value31865);
      Serial.println(")");
#ifndef NO_DISPLAY
      ::gos::atl::format::error(displaybuffer, error31865, length31865);
#endif
    }
#ifndef NO_DISPLAY
    displayoneline.display(displaybuffer);
#endif
  }

#ifndef NO_DISPLAY
  displayoneline.loop();
#endif
}
