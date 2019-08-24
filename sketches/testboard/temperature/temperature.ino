/* Build Options */
//#define NO_DISPLAY

#include <SPI.h>

#include <gosmax6675.h>
#include <gosmax31865.h>
#include <arduinosensor.h>
#include <gatlmedian.h>

#define TEXT_UNIT " C"

#ifndef NO_DISPLAY
#include <gatldisplay.h>
#include <gatlformat.h>
::gos::atl::format::option::Number displayoption;
::gos::atl::buffer::Holder<> displayunit(TEXT_UNIT, sizeof(TEXT_UNIT));
::gos::atl::display::Oled<> oled;
::gos::atl::display::line::Two<> displaytwoline(oled);
bool isdisplaybufferupdated;
#endif

#include "fds-celsius-logo.h"
#include "temperature.h"
#include "modbus.h"

#define PIN_SPI_MAX_31865_CS        7
#define PIN_SPI_MAX_6675_CS         8

#define DELAY_SENSOR_SETUP_END    500

//#define SERIAL_BAUD            115200

/* For PT100 set type to 1 and for PT1000 set type to 2 */
#define MAX_RTD_TYPE  RTD_TYPE_PT1000

// set to 2WIRE or 4WIRE as necessary
#define MAX_WIRES          RTD_3_WIRE

unsigned long tick;

::gos::Max31865 max31865(PIN_SPI_MAX_31865_CS);
::gos::Max6675 max6675(PIN_SPI_MAX_6675_CS);

void reset(Sensor& sensor);
void check(Sensor& sensor);
bool isok(const Sensor& sensor);
void output(const Sensor& sensor);
#ifndef NO_DISPLAY
void display(const Sensor& sensor);
#endif

void setup() {
#ifndef NO_DISPLAY
  oled.U8g2->begin();
  oled.logo(
    fds_celsius_logo_width,
    fds_celsius_logo_height,
    fds_celsius_logo_bits);
#endif

  SPI.begin();

  max31865.initialize(MAX_RTD_TYPE, MAX_WIRES);
  max6675.initialize();

#if   defined(MODBUS_BAUD)
  ::gos::modbus::begin();
#elif defined(SERIAL_BAUD)
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
    delay(10);
  }
  Serial.println("MAX 31865 and 6675 testing");
#endif

#ifdef DELAY_SENSOR_SETUP_END
  delay(DELAY_SENSOR_SETUP_END);
#endif
}

void loop() {
  tick = millis();

#ifndef NO_DISPLAY
  isdisplaybufferupdated = false;
#endif

  if (sensormax31865.Timer.is(tick)) {
    reset(sensormax31865);
    if (max31865.read(sensormax31865.Value)) {
      check(sensormax31865);
    }
    output(sensormax31865);
#ifndef NO_DISPLAY
    display(sensormax31865);
    isdisplaybufferupdated = true;
#endif
  }

  if (sensormax6675.Timer.is(tick)) {
    reset(sensormax6675);
    if (max6675.read(sensormax6675.Value)) {
      check(sensormax6675);
    }
    output(sensormax6675);
#ifndef NO_DISPLAY
    display(sensormax6675);
    isdisplaybufferupdated = true;
#endif
  }

#ifndef NO_DISPLAY
  if (isdisplaybufferupdated) {
    displaytwoline.display(
      sensormax31865.Display.Buffer,
      sensormax6675.Display.Buffer);
  }
  displayoneline.loop();
#endif
}

void reset(Sensor& sensor) {
  sensor.Error.Text = nullptr;
  sensor.Error.Length = 0;
  sensor.Display.Buffer[0] = '\0';
}

void check(Sensor& sensor) {
  if (::gos::sensor::range::check(sensor.Value) == GOS_SENSOR_STATUS_OK) {
#ifndef NO_STATISTICS
    sensor.Set.add(sensor.Value);
#endif
  }
  else {
    sensor.Error.Text = gos::sensor::error(sensor.Error.Length);
  }
}

bool isok(const Sensor& sensor) {
  return sensor.Error.Text == nullptr && sensor.Error.Length == 0;
}

void output(const Sensor& sensor) {
  Serial.print(sensor.DisplayId.Buffer);
  Serial.print(" ");
  if (isok(sensor)) {
    Serial.print(sensor.Value);
    Serial.println(TEXT_UNIT);
  }
  else {
    Serial.println(sensor.Error.Text);
  }
}

#ifndef NO_DISPLAY
void display(const Sensor& sensor) {
  if (isok(sensor)) {
    ::gos::atl::format::real(
      sensor.Display.Buffer,
      sensor.Value,
      displayoption,
      &sensor.Display.Id,
      &displayunit);
  }
  else {
    ::gos::atl::format::error(
      sensor.Display.Buffer,
      sensor.Error.Text,
      sensor.Error.Length,
      &sensor.Display.Id);
    )
  }
}
#endif
