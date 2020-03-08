#ifndef GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_DISPLAY_H_
#define GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_DISPLAY_H_

#include <gatldisplay.h>

#ifndef NO_DISPLAY

namespace gos {
namespace temperature {
namespace display {

typedef ::gos::atl::display::Oled<> Oled;
typedef ::gos::atl::display::asynchronous::line::Two<> Two;
extern Oled oled;
extern Two two;

} // namespace display
} // namespace temperature
} // namespace gos

#endif
#endif
