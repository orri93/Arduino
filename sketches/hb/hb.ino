#include <gatlmedian.h>
#include <gatlavrage.h>

#define PIN_ANALOG_READ A0

#define SET_UNDEFINED 0
#define SET_COUNT    20

::gos::atl::statistics::Set<int> set(SET_UNDEFINED, SET_COUNT);
::gos::atl::statistics::Median<int> median(set);
::gos::atl::statistics::Avrage<int> avrage(set);

void setup() {
  pinMode(PIN_ANALOG_READ, INPUT);
}

void loop() {
  avrage.add(analogRead(PIN_ANALOG_READ));

  if(set.Count)
}
