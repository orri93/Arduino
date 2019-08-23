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
#include <gatlmedian.h>

#ifndef NO_DISPLAY
#include <gatldisplay.h>
#include <gatlformat.h>

#define TEXT_ID_IMMEDIATE "T: "
#define TEXT_ID_MEDIAN "M: "


#define TEXT_UNIT " C"

::gos::atl::format::option::Number displayoption;
::gos::atl::buffer::Holder<> displaybufferimmediate;
::gos::atl::buffer::Holder<> displaybuffermedian;
::gos::atl::buffer::Holder<> idimmediate(TEXT_ID_IMMEDIATE, sizeof(TEXT_ID_IMMEDIATE));
::gos::atl::buffer::Holder<> idmedian(TEXT_ID_MEDIAN, sizeof(TEXT_ID_MEDIAN));
::gos::atl::buffer::Holder<> displayunit(TEXT_UNIT, sizeof(TEXT_UNIT));
::gos::atl::display::Oled<> oled;
::gos::atl::display::line::Two<> displaytwoline(oled);
#endif

#define PIN_MAX6675_CS           8

#define INTERVAL_MAX_6675     2000

#define DELAY_SENSOR_SETUP_END 500

#define SERIAL_BAUD          19200

Tick timermax6675(INTERVAL_MAX_6675);

::gos::atl::statistics::Set<double> set(NAN);
::gos::atl::statistics::Median<double> median(set);

::gos::Max6675 max6675(PIN_MAX6675_CS);

unsigned long tick;

const char* error6675;
uint8_t status6675, length6675;
double value6675, median6675;

void setup() {
#ifndef NO_DISPLAY
  oled.U8g2->begin();
  displaybuffermedian.Buffer[0] = '\0';
#endif

  SPI.begin();

  max6675.initialize();

#ifdef SERIAL_BAUD
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
    delay(10);
  }
  Serial.println("MAX 6675 testing");
#endif

#ifdef DELAY_SENSOR_SETUP_END
  delay(DELAY_SENSOR_SETUP_END);
#endif
}

void loop() {
  tick = millis();

  if (timermax6675.is(tick)) {
    error6675 = nullptr;
    if (max6675.read(value6675)) {
      if (::gos::sensor::range::check(value6675) == GOS_SENSOR_STATUS_OK) {
        set.add(value6675);
      }
      else {
        error6675 = ::gos::sensor::error(length6675);
      }
    }
    else {
      error6675 = max6675.error(length6675);
    }
    if (error6675 == nullptr) {
      Serial.print(value6675, 1);
      if (set.Count > 6) {
        median6675 = median.get();
        Serial.print(",");
        Serial.println(median6675, 1);
      }
      Serial.println();
#ifndef NO_DISPLAY
      ::gos::atl::format::real(
        displaybufferimmediate,
        value6675,
        displayoption,
        &idimmediate,
        &displayunit);
      if (set.Count > 6) {
        ::gos::atl::format::real(
          displaybuffermedian,
          median6675,
          displayoption,
          &idmedian,
          &displayunit);
      }
#endif
    }
    else {
      Serial.println(error6675);
#ifndef NO_DISPLAY
      ::gos::atl::format::error(
        displaybufferimmediate,
        error6675,
        length6675,
        &idimmediate);
      displaybuffermedian.Buffer[0] = '\0';
#endif
    }
#ifndef NO_DISPLAY
    displaytwoline.display(displaybufferimmediate, displaybuffermedian);
#endif
  }

#ifndef NO_DISPLAY
  displaytwoline.loop();
#endif
}
