#include <arduinotick.h>

#include <gatlmedian.h>

#define PIN_PUSH_BUTTON_A        4
#define PIN_PUSH_BUTTON_B        7
#define PIN_PUSH_BUTTON_C        8

#define PIN_LED_GREEN            3
#define PIN_LED_YELLOW           4
#define PIN_LED_RED              6
#define PIN_LED_BLUE             5

#define INTERVAL_CYCLE        1000

#define SERIAL_BAUD          19200

namespace gatl = ::gos::atl;

namespace push {
namespace button {
int a = 0, b = 0, c = 0;
bool is(const uint8_t& pin, int& count, const int& threshold = 32) {
  return (count = (digitalRead(pin) == LOW ?
    (count < threshold ? count + 1  : count) :
    (count > 0 ? count - 1 : 0))) >= threshold;
}
}
}

void setup() {
  pinMode(PIN_PUSH_BUTTON_A, INPUT_PULLUP);
  pinMode(PIN_PUSH_BUTTON_B, INPUT_PULLUP);
  pinMode(PIN_PUSH_BUTTON_C, INPUT_PULLUP);
  pinMode(PIN_LED_RED_A, OUTPUT);
  pinMode(PIN_LED_BLUE_A, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  digitalWrite(PIN_LED_RED_A, LOW);
  digitalWrite(PIN_LED_BLUE_A, LOW);
  digitalWrite(PIN_LED_GREEN, LOW);
}

void loop() {
  digitalWrite(
    PIN_LED_RED_A,
    push::button::is(PIN_PUSH_BUTTON_A, push::button::a) ? HIGH : LOW);
  digitalWrite(
    PIN_LED_BLUE_A,
    push::button::is(PIN_PUSH_BUTTON_B, push::button::b) ? HIGH : LOW);
  digitalWrite(
    PIN_LED_GREEN,
    push::button::is(PIN_PUSH_BUTTON_C, push::button::c) ? HIGH : LOW);
}