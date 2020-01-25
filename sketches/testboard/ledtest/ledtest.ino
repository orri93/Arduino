#include <gatled.h>

#define PIN_LED_1       3
#define PIN_LED_2       4
#define PIN_LED_3       5
#define PIN_LED_4       6

#define PIN_BUTTON_A    7
#define PIN_BUTTON_B    8

#define PIN_LED_5       5
#define PIN_LED_6       6

float led3 = 0.0F, led4 = 0.0F, led5 = 0.0F, led6 = 0.0F;

namespace gatl = ::gos::atl;
namespace gatll = ::gos::atl::led;

void setup() {
  gatll::initialize(PIN_LED_1);
  gatll::initialize(PIN_LED_2);
  gatll::initialize(PIN_LED_3);
  gatll::initialize(PIN_LED_4);
  gatll::initialize(PIN_LED_5);
  gatll::initialize(PIN_LED_6);

  gatll::blink<>(PIN_LED_1);
  gatll::blink<>(PIN_LED_2);
}

void loop() {
}
