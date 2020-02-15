#include <gatlstring.h>

#include "format.h"
#include "display.h"

namespace gatl = ::gos::atl;
namespace gm = ::gos::modbus;

namespace gos {
namespace modbus {

namespace display {
Oled oled;
Two two(oled);
namespace update {
namespace first {

void line(const char* text) {
  if (gatl::string::compare(gm::format::display::buffer::first, text) != 0) {
    gatl::string::copy(gm::format::display::buffer::first, text);
    gm::display::two.display(
      gm::format::display::buffer::first,
      gm::format::display::buffer::second);
  }
}

} // namespace first
namespace second {

void line(const char* text) {
  if (gatl::string::compare(gm::format::display::buffer::second, text) != 0) {
    gatl::string::copy(gm::format::display::buffer::second, text);
    gm::display::two.display(
      gm::format::display::buffer::first,
      gm::format::display::buffer::second);
  }
}

void line() {
}

} // namespace second
} // namespace update
} // namespace display

} // namespace modbus
} // namespace gos
