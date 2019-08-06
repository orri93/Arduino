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

unsigned tick, timetocheckbutton = 0;

::fds::SinLed sinledr(PIN_RGB_LED_R);
::fds::SinLed sinledg(PIN_RGB_LED_G);
::fds::SinLed sinledb(PIN_RGB_LED_B);

void setup() {
  pinMode(PIN_BUTTON_RGB, INPUT);
  pinMode(PIN_BUTTON_FUN, INPUT);

  sinledr.initialize();
  sinledg.initialize();
  sinledb.initialize();
}

bool isonr = false, isong = false, isonb = false;

void loop() {
  tick = millis();
  
  if(tick >= timetocheckbutton) {
    timetocheckbutton = tick + INTERVAL_BUTTON;
    if(counta >= BUTTON_THRESHOLD) {
      if(isonr) {
        if(isong) {
          isonb = true;
        } else {
          isong = true;
        }
      } else {
        isonr = true;
      }
    }
    if(countb >= BUTTON_THRESHOLD && !isonb) {
      if(!isonb) {
        if(!isong) {
          isonr = false;
        } else {
          isong = false;
        }
      } else {
        isonb = false;
      }
    }
    counta = countb = 0;
  }

  if(isonr) {
    sinledr.cycle();
  }
  if(isong) {
    sinledg.cycle();
  }
  if(isonb) {
    sinledb.cycle();
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
