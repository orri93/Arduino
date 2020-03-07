#include <Arduino.h>

#define PIN_RS485_RX                         0
#define PIN_RS485_TX                         1
#define PIN_RS485_TE                         2

#define PIN_LED_RED_A                        3  /* ~ 490 Hz PWM */
#define PIN_LED_RED_B                       11  /* ~ 490 Hz PWM */

#define PIN_LED_BLUE_A                       6  /* ~ 980 Hz PWM */
#define PIN_LED_BLUE_B                       5  /* ~ 980 Hz PWM */

#define PIN_LED_GREEN                        9  /* ~ 490 Hz PWM */
#define PIN_LED_YELLOW                      10  /* ~ 490 Hz PWM */

#define PIN_LED_MODBUS_READ      PIN_LED_GREEN
#define PIN_LED_MODBUS_WRITE    PIN_LED_YELLOW

#define PIN_BUTTON_A                         8
#define PIN_BUTTON_B                         7

#define PIN_POTENTIOMETER                   A0

#define RS485_BAUD                        9600

#define BUFFER_SIZE                        128

const uint8_t leds[4] = {
  PIN_LED_RED_A,
  PIN_LED_RED_B,
  PIN_LED_BLUE_A,
  PIN_LED_BLUE_B
};

int r;
uint8_t buffer[BUFFER_SIZE];
uint8_t i, count, lb, hb, fhb, shb;
uint16_t w;

void setup() {
  pinMode(PIN_RS485_TE, OUTPUT);
  pinMode(PIN_LED_RED_A, OUTPUT);
  pinMode(PIN_LED_RED_B, OUTPUT);
  pinMode(PIN_LED_BLUE_A, OUTPUT);
  pinMode(PIN_LED_BLUE_B, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_YELLOW, OUTPUT);
  pinMode(PIN_BUTTON_A, INPUT_PULLUP);
  pinMode(PIN_BUTTON_B, INPUT_PULLUP);
  Serial.begin(RS485_BAUD);
}

void loop() {
  count = 0;
  
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  while((r = Serial.read()) >= 0 && count < BUFFER_SIZE) {
    buffer[count++] = (uint8_t)r;
  }
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  if (count > 0) {

    /* Parse LED instructions */
    i = count;
    do {
      --i;
      if (i < 4) {
        analogWrite(leds[i], buffer[i]);
      }
    } while (i != 0);

    /* Create output */
    buffer[0] = 0x20 &
      ((digitalRead(PIN_BUTTON_A) == HIGH) ? 0x01 : 0x00) &
      ((digitalRead(PIN_BUTTON_B) == HIGH) ? 0x02 : 0x00);

    w = analogRead(PIN_POTENTIOMETER);

    uint8_t lb = lowByte(w);
    uint8_t hb = highByte(w);

    buffer[1] = lb & 0x0f;
    buffer[2] = lb >> 4;
    buffer[3] = hb & 0x0f;
    buffer[4] = hb >> 4;

    /* Send response */
    count = 5;
    digitalWrite(PIN_RS485_TE, HIGH);
    digitalWrite(PIN_LED_MODBUS_WRITE, HIGH);
    for (i = 0; i < count; ++i) {
      Serial.write(buffer[i]);
    }
    digitalWrite(PIN_LED_MODBUS_WRITE, LOW);
    digitalWrite(PIN_RS485_TE, LOW);
  }
}
