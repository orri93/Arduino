#include <gatled.h>

#define PIN_LED_1       3
#define PIN_LED_2       4
#define PIN_LED_3       5
#define PIN_LED_4       6

#define PIN_BUTTON_A    7
#define PIN_BUTTON_B    8

#define PIN_LED_5       5
#define PIN_LED_6       6

fds::SinLed led_green(PIN_LED_GREEN);
fds::SinLed led_yellow(PIN_LED_YELLOW);
fds::SinLed led_blue(PIN_LED_BLUE);
fds::SinLed led_red(PIN_LED_RED);

float led3 = 0.0F, led4 = 0.0F, led5 = 0.0F, led6 = 0.0F;

namespace gatl = ::gos::atl;
namespace gatlL = ::gos::atl::led;


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
