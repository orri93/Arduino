#include <gatldisplay.h>
#include <gatlformat.h>
#include <arduinotick.h>

#include "fds-logo.h"

#define INTERVAL 5000

#define TEXT_ID_A "A: "
#define TEXT_ID_B "B: "
#define TEXT_UNIT " U"

namespace ga = ::gos::atl;
namespace gad = ::gos::atl::display;

Tick timer(INTERVAL);

namespace number {
typedef ga::format::option::Number Option;
}
typedef ga::buffer::Holder<> Holder;
typedef ga::display::Oled<> Oled;
namespace display {
namespace line {
typedef ga::display::asynchronous::line::One<> One;
typedef ga::display::asynchronous::line::Two<> Two;
}
namespace render {
typedef ga::display::asynchronous::Render<>* Pointer;
}
}

number::Option option;
Holder buffer1;
Holder buffer2;
Holder ida(TEXT_ID_A, sizeof(TEXT_ID_A));
Holder idb(TEXT_ID_B, sizeof(TEXT_ID_B));
Holder unit(TEXT_UNIT, sizeof(TEXT_UNIT));
Oled oled;
display::line::One oneline(oled);
display::line::Two twoline(oled);
display::render::Pointer render = &oneline;

unsigned long tick;

double real;
int integer;

enum class Lines {
  One, Two
};

Lines lines = Lines::One;

void setup() {
  oled.U8g2->begin();
  ga::display::synchronous::logo(
    oled,
    fds_logo_width,
    fds_logo_height,
    fds_logo_bits);
  delay(INTERVAL);
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
