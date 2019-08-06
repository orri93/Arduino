#include <arduinosinled.h>

#define BUTTON_THRESHOLD          0x55

#define PIN_BUTTON_RGB            0x02
#define PIN_BUTTON_FUN            0x03

#define PIN_RGB_LED_R             0x00
#define PIN_RGB_LED_G             0x01
#define PIN_RGB_LED_B             0x04

#define BIT_R                     0x01
#define BIT_G                     0x02
#define BIT_B                     0x04

#define BIT_33P                   0x10
#define BIT_66P                   0x20
#define BIT_99P                   0x40
#define BIT_SIN                   0x80

#define LED_OFF                   0x00
#define LED_R                    BIT_R
#define LED_G                    BIT_G
#define LED_B                    BIT_B
#define LED_RG           BIT_R | BIT_G
#define LED_GB           BIT_G | BIT_B
#define LED_RB           BIT_R | BIT_B
#define LED_RGB  BIT_R | BIT_G | BIT_G

#define FUN_99P                BIT_99P
#define FUN_66P                BIT_66P
#define FUN_33P                BIT_33P
#define FUN_99P_SIN  BIT_SIN | BIT_SIN
#define FUN_66P_SIN  BIT_SIN | BIT_66P
#define FUN_33P_SIN  BIT_SIN | BIT_33P

#define INTERVAL_BUTTON           0xff

#define LED_ORDER_COUNT              8
#define FUN_ORDER_COUNT              6

uint8_t ledorder[] = {
  LED_RGB,
  LED_OFF,
  LED_R,
  LED_G,
  LED_B,
  LED_RG,
  LED_GB,
  LED_RB
};

uint8_t funorder[] = {
  FUN_99P,
  FUN_66P,
  FUN_33P,
  FUN_99P_SIN,
  FUN_66P_SIN,
  FUN_33P_SIN,
};

uint8_t led = 0, fun = 0;
uint8_t ledind = 0, funind = 0;
uint8_t button, countrgb = 0, countfun = 0;

uint8_t r = 0, g = 0, b = 0;

unsigned tick, timetocheckbutton = 0, timetorun = 0, timetosin = 0;

::fds::SinLed sinledr(PIN_RGB_LED_R);
::fds::SinLed sinledg(PIN_RGB_LED_G);
::fds::SinLed sinledb(PIN_RGB_LED_B);

uint8_t rgb(
  const uint8_t& pin,
  const uint8_t& fun,
  const uint8_t& led,
  const uint8_t& bit,
  const uint8_t& last,
  ::fds::SinLed& sinled) {
  uint8_t result = 0;
  if((led & bit) == bit) {
    result = 0x00;
    if((fun & BIT_99P) == BIT_99P) {
      result = 0xff;
    } else if((fun & BIT_66P) == BIT_66P) {
      result = 0xaa;
    } else if((fun & BIT_33P) == BIT_33P) {
      result = 0x55;
    }
    if((fun & BIT_SIN) == BIT_SIN) {
      result = sinled.cycle(result);
    } else if(last != result) {
      if(result < 0xff) {
        analogWrite(pin, result);;
      } else {
        digitalWrite(pin, HIGH);
      }
    }
  } else {
    if(last > 0) {
      digitalWrite(pin, LOW);
    }
  }
  return result;
}

void setup() {
  pinMode(PIN_BUTTON_RGB, INPUT);
  pinMode(PIN_BUTTON_FUN, INPUT);

  sinledr.initialize(2, 100);
  sinledg.initialize(2, 100);
  sinledb.initialize(2, 100);
}

void loop() {
  tick = millis();
  
  if(tick >= timetocheckbutton) {
    timetocheckbutton = tick + INTERVAL_BUTTON;
    if(countrgb >= BUTTON_THRESHOLD) {
      ledind++;
      if(ledind >= LED_ORDER_COUNT) {
        ledind = 0;
      }
    }
    if(countfun >= BUTTON_THRESHOLD) {
      funind++;
      if(funind >= FUN_ORDER_COUNT) {
        funind = 0;
      }
    }
    countrgb = 0;
    countfun = 0;
  }

  led = ledorder[ledind];
  fun = funorder[funind];

  r = rgb(PIN_RGB_LED_R, fun, led, BIT_R, r, sinledr);
  g = rgb(PIN_RGB_LED_G, fun, led, BIT_G, g, sinledg);
  b = rgb(PIN_RGB_LED_B, fun, led, BIT_B, b, sinledb);

  button = digitalRead(PIN_BUTTON_RGB);
  if(button == LOW && countrgb < 0xff) {
    countrgb++;
  }
  button = digitalRead(PIN_BUTTON_FUN);
  if(button == LOW && countfun < 0xff) {
    countfun++;
  }
}
