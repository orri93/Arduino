#ifndef GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_MODBUS_DISPLAY_H_
#define GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_MODBUS_DISPLAY_H_

#include <gatldisplay.h>

namespace gos {
namespace modbus {

namespace display {
typedef ::gos::atl::display::Oled<> Oled;
typedef ::gos::atl::display::asynchronous::line::Two<> Two;
extern Oled oled;
extern Two two;
extern bool updated;
namespace update {
namespace first {
void line(const char* text);
} // namespace first
namespace second {
void line(const char* text);
void line();
} // namespace second
} // namespace update
} // namespace display

} // namespace modbus
} // namespace gos

#endif
