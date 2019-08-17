#include <arduinosinled.h>

#define PIN_LED_GREEN   3
#define PIN_LED_YELLOW  4
#define PIN_LED_BLUE    5
#define PIN_LED_RED     6

fds::SinLed led_green(PIN_LED_GREEN);
fds::SinLed led_yellow(PIN_LED_YELLOW);
fds::SinLed led_blue(PIN_LED_BLUE);
fds::SinLed led_red(PIN_LED_RED);

void setup() {
  led_green.initialize();
  led_yellow.initialize();
  led_blue.initialize();
  led_red.initialize();
}

void loop() {
  led_green.cycle();
  led_yellow.cycle();
  led_blue.cycle();
  led_red.cycle();
}
