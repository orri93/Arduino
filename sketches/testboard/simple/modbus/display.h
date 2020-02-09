#ifndef GOS_ARDUINO_SKETCHES_TESTBOARD_SIMPLE_MODBUS_DISPLAY_H_
#define GOS_ARDUINO_SKETCHES_TESTBOARD_SIMPLE_MODBUS_DISPLAY_H_

#include <gatldisplay.h>

namespace gos {
namespace modbus {

namespace display {
extern ::gos::atl::display::Oled<> oled;
extern ::gos::atl::display::asynchronous::line::Two<> twoline;
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
