#include <gatlled.h>

#define PIN_LED_RED_A         3
#define PIN_LED_RED_B        11

#define PIN_LED_BLUE_A        6
#define PIN_LED_BLUE_B        5

#define PIN_LED_GREEN         9
#define PIN_LED_YELLOW       10

#define PIN_BUTTON_A          8
#define PIN_BUTTON_B          7

#define PIN_POTENTIOMETER    A0

float led3 = 0.0F, led4 = 0.0F, led5 = 0.0F, led6 = 0.0F;

namespace gatl = ::gos::atl;
namespace gatll = ::gos::atl::led;

void setup() {
  gatll::initialize(PIN_LED_RED_A);
  gatll::initialize(PIN_LED_RED_B);
  gatll::initialize(PIN_LED_BLUE_A);
  gatll::initialize(PIN_LED_BLUE_B);
  gatll::initialize(PIN_LED_GREEN);
  gatll::initialize(PIN_LED_YELLOW);

  gatll::blink<>(PIN_LED_RED_A);
  gatll::blink<>(PIN_LED_RED_B);
  gatll::blink<>(PIN_LED_BLUE_A);
  gatll::blink<>(PIN_LED_BLUE_B);
  gatll::blink<>(PIN_LED_GREEN);
  gatll::blink<>(PIN_LED_YELLOW);
}

void loop() {
}
