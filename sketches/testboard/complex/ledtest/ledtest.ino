#include <gatlled.h>

#ifdef LED_SEARCH
#define PIN_LED_03            3
#define PIN_LED_04            4
#define PIN_LED_05            5
#define PIN_LED_06            6
#define PIN_LED_07            7
#define PIN_LED_08            8
#define PIN_LED_09            9
#define PIN_LED_10           10
#define PIN_LED_11           11
#define PIN_LED_12           12
#define PIN_LED_13           13
#else
#define PIN_LED_GREEN         3
#define PIN_LED_YELLOW        4
#define PIN_LED_RED           6
#define PIN_LED_BLUE          5
#endif

#define PIN_POTENTIOMETER    A0

float led3 = 0.0F, led4 = 0.0F, led5 = 0.0F, led6 = 0.0F;

namespace gatl = ::gos::atl;
namespace gatll = ::gos::atl::led;

void setup() {
#ifdef LED_SEARCH
  gatll::initialize(PIN_LED_03);
  gatll::initialize(PIN_LED_04);
  gatll::initialize(PIN_LED_05);
  gatll::initialize(PIN_LED_06);
  gatll::blink<>(PIN_LED_03);
  gatll::blink<>(PIN_LED_04);
  gatll::blink<>(PIN_LED_05);
  gatll::blink<>(PIN_LED_06);
#else
  gatll::initialize(PIN_LED_GREEN);
  gatll::initialize(PIN_LED_YELLOW);
  gatll::initialize(PIN_LED_RED);
  gatll::initialize(PIN_LED_BLUE);
  gatll::blink<>(PIN_LED_GREEN);
  gatll::blink<>(PIN_LED_YELLOW);
  gatll::blink<>(PIN_LED_RED);
  gatll::blink<>(PIN_LED_BLUE);
#endif
}

void loop() {
}
