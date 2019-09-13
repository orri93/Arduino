#include <gatlmedian.h>
#include <gatlavrage.h>

#include <arduinotick.h>

#define PIN_ANALOG_READ A2

#define SERIAL_BAUD_RATE 115200

#define SET_UNDEFINED 0
#define SET_COUNT    20

//#define INTERVAL     10

::gos::atl::statistics::Set<int> set(SET_UNDEFINED, SET_COUNT);
::gos::atl::statistics::Median<int> median(set);
::gos::atl::statistics::Avrage<int> avrage(set);

#ifdef INTERVAL
Tick timer(INTERVAL);
#endif

int v, m = 0, a = 0;

void setup() {
  pinMode(PIN_ANALOG_READ, INPUT);
  Serial.begin(SERIAL_BAUD_RATE);
}

void loop() {
  v = analogRead(PIN_ANALOG_READ);
  avrage.add(v);

  if (set.issaturated()) {
    m = median.get();
    a = avrage.get();

    Serial.print(v, DEC);
    Serial.print(",");
    Serial.print(m, DEC);
    Serial.print(",");
    Serial.print(a, DEC);
    Serial.println();

  }
}
