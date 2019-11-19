#include <arduinotick.h>

#define PIN_PUSH_BUTTON                  A0

#define INTERVAL_CYCLE                 1000

#define SERIAL_BAUD                   19200

#define PUSH_BUTTON_ANALOG_THREASHOLD   512
#define PUSH_BUTTON_LONG_PRESS_TIME     250

namespace push {
namespace button {
enum class status { undefined, a, b ,c };
uint16_t value;
namespace active {
bool is = false;
bool islong = false;
unsigned long time;
}
unsigned long tick;
Tick timer(INTERVAL_CYCLE);
}
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(PIN_PUSH_BUTTON, INPUT_PULLUP);
}

void loop() {
  push::button::tick = millis();
  push::button::value = analogRead(PIN_PUSH_BUTTON);

  if (push::button::value < PUSH_BUTTON_ANALOG_THREASHOLD) {
    if (!push::button::active::is) {
      push::button::active::is = true;
      push::button::active::time = push::button::tick;
    }
    if ((push::button::tick - push::button::active::time) > PUSH_BUTTON_LONG_PRESS_TIME && !push::button::active::islong) {
      push::button::active::islong = true;
      Serial.println("Long press");
    }
  } else {
    if (push::button::active::is) {
      if (push::button::active::islong) {
        push::button::active::islong = false;
      } else {
        Serial.println("Short press");
      }
      push::button::active::is = false;
    }
  }
}