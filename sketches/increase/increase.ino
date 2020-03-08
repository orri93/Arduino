#include <Arduino.h>

#define INCREASE_BAUD 9600
#define BUFFER_SIZE 128

size_t i, count;

char buffer[BUFFER_SIZE];

void setup() {
  Serial.begin(INCREASE_BAUD);
  Serial.write("Send a text and the respond will be "
    "the text with each char increased by one\n");
}

void loop() {
  count = Serial.readBytesUntil('\n', buffer, BUFFER_SIZE);
  if (count > 0) {
    for (i = 0; i < count; i++) {
      Serial.write(buffer[i] < 255 ? buffer[i] + 1 : 255);
    }
  }
}
