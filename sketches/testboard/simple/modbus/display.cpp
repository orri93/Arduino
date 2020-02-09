#include "format.h"
#include "display.h"

namespace gatl = ::gos::atl;
namespace gm = ::gos::modbus;

namespace gos {
namespace modbus {

namespace display {
gatl::display::Oled<> oled;
gatl::display::asynchronous::line::Two<> twoline(oled);
bool updated = false;
namespace update {
namespace first {

void line(const char* text) {
  gatl::buffer::strncpy(gm::format::display::buffer::first, text);
  updated = false;
}

} // namespace first
namespace second {

void line(const char* text) {
  gatl::buffer::strncpy(gm::format::display::buffer::second, text);
}

void line() {
}

} // namespace second
} // namespace update
} // namespace display

} // namespace modbus
} // namespace gos
