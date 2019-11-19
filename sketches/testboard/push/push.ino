#include <arduinotick.h>

#include <gatlmedian.h>

#define PIN_PUSH_BUTTON         A0

#define INTERVAL_CYCLE        1000

#define SERIAL_BAUD          19200

namespace gatl = ::gos::atl;

namespace push {
namespace button {
uint16_t value;
gatl::statistics::Set<float> set(NAN);
gatl::statistics::Median<float> median(set);
float m = 0.0F;
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
  push::button::set.add(push::button::value);
  if (push::button::timer.is(push::button::tick)) {
    Serial.print(push::button::value, 0);
    Serial.print(",");
    if (push::button::set.Count > 6) {
      push::button::m = push::button::median.get();
    }
    Serial.print(push::button::m, 1);
    Serial.print(",");
    Serial.println();
  }
}