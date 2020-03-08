#include <Arduino.h>

#define PIN_LED_RED              6
#define PIN_LED_BLUE             5

uint16_t analog;

void setup() {
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  digitalWrite(PIN_LED_RED, LOW);
  digitalWrite(PIN_LED_BLUE, LOW);
}

void loop() {
  analog = analogRead(A0);
  analogWrite(PIN_LED_RED, map(analog, 0, 1024, 0, 255));
  analog = analogRead(A2);
  analogWrite(PIN_LED_BLUE, map(analog, 0, 1024, 0, 255));
}
