#include <Arduino.h>

#define REVERSE_BAUD 9600
#define BUFFER_SIZE 128

uint8_t at;
int read, i;

char buffer[BUFFER_SIZE];

void setup() {
  Serial.begin(REVERSE_BAUD);
  Serial.write("Send a text and the respond will be the text reversed\n");
}

void loop() {
  at = 0;
  while (at < BUFFER_SIZE && (read = Serial.read()) >= 0) {
    buffer[at++] = (char)read;
  }
  for (i = at - 1; i >= 0; --i) {
    Serial.write(buffer[i]);
  }
}
