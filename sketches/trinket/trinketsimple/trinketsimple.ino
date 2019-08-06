#include <arduinosinled.h>

#define BUTTON_THRESHOLD          0x10

#define PIN_BUTTON_RGB            0x02
#define PIN_BUTTON_FUN            0x03

#define PIN_RGB_LED_R             0x00
#define PIN_RGB_LED_G             0x01
#define PIN_RGB_LED_B             0x04

#define INTERVAL_BUTTON             50
#define DURATION                  1000

uint8_t button, counta = 0, countb = 0;

unsigned tick, timetocheckbutton = 0, timetooffa, timetooffb;

bool isona = false, isonb = false, turnona = false, turnonb = false;

::fds::SinLed sinledr(PIN_RGB_LED_R);
::fds::SinLed sinledg(PIN_RGB_LED_G);
::fds::SinLed sinledb(PIN_RGB_LED_B);

void setup() {
  pinMode(PIN_BUTTON_RGB, INPUT);
  pinMode(PIN_BUTTON_FUN, INPUT);

  sinledr.initialize(16, 10);
  sinledg.initialize(16, 10);
  sinledb.initialize(16, 10);
}

void loop() {
  tick = millis();

  if(isona & tick >= timetooffa) {
    digitalWrite(PIN_RGB_LED_R, LOW);
    isona = false;
  }
  if(isonb & tick >= timetooffb) {
    digitalWrite(PIN_RGB_LED_B, LOW);
    isonb = false;
  }
  
  if(tick >= timetocheckbutton) {
    timetocheckbutton = tick + INTERVAL_BUTTON;
    if(counta >= BUTTON_THRESHOLD && !isona) {
      turnona = true;
    }
    if(countb >= BUTTON_THRESHOLD && !isonb) {
      turnonb = true;
    }
    counta = countb = 0;
  }

  if(turnona) {
    digitalWrite(PIN_RGB_LED_R, HIGH);
    timetooffa = tick + DURATION;
    turnona = false;
    isona = true;
  }
  if(turnonb) {
    digitalWrite(PIN_RGB_LED_B, HIGH);
    timetooffb = tick + DURATION;
    turnonb = false;
    isonb = true;
  }

  button = digitalRead(PIN_BUTTON_RGB);
  if(button == LOW) {
    counta++;
  }
  button = digitalRead(PIN_BUTTON_FUN);
  if(button == LOW) {
    countb++;
  }
}
