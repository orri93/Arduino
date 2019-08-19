#include <gatldisplay.h>
#include <gatlformat.h>
#include <arduinotick.h>

#define INTERVAL 5000

#define TEXT_ID_A "A: "
#define TEXT_ID_B "B: "
#define TEXT_UNIT " C"

Tick timer(INTERVAL);

::gos::atl::format::option::Number option;
::gos::atl::buffer::Holder<> buffer1;
::gos::atl::buffer::Holder<> buffer2;
::gos::atl::buffer::Holder<> ida(TEXT_ID_A, sizeof(TEXT_ID_A));
::gos::atl::buffer::Holder<> idb(TEXT_ID_B, sizeof(TEXT_ID_B));
::gos::atl::buffer::Holder<> unit(TEXT_UNIT, sizeof(TEXT_UNIT));
::gos::atl::display::Oled<> oled;
::gos::atl::display::line::One<> oneline(oled);
::gos::atl::display::line::Two<> twoline(oled);
::gos::atl::display::Render<>* render = &oneline;

unsigned long tick;

double real;
int integer;

enum class Lines {
  One, Two
} lines = Lines::One;

void setup() {
  oled.U8g2->begin();
}

void loop() {
  tick = millis();

  if (timer.is(tick)) {
    switch (lines) {
    case Lines::One:
      integer = random(255);
      ::gos::atl::format::integer(buffer1, integer, &ida, &unit);
      oneline.display(buffer1);
      render = &oneline;
      lines = Lines::Two;
      break;
    case Lines::Two:
      real = (double)(random(1024)) / 16.0;
      ::gos::atl::format::real(buffer1, real, option, &ida, &unit);
      real = (double)(random(1024)) / 16.0;
      ::gos::atl::format::real(buffer2, real, option, &idb, &unit);
      twoline.display(buffer1, buffer2);
      lines = Lines::One;
      render = &twoline;
      break;
    }
  }

  render->loop();
}
