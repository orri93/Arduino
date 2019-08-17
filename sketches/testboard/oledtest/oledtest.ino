#include <arduinodisplay.h>
#include <arduinoformat.h>
#include <arduinotick.h>

#define INTERVAL 5000

Tick timer(INTERVAL);

unsigned long tick;

void setup() {
  fds::display::u8g2::begin();
}

void loop() {
  fds::display::u8g2::
}
