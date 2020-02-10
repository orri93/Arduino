#include <gatlstring.h>

#include "format.h"
#include "display.h"

namespace gatl = ::gos::atl;
namespace gm = ::gos::modbus;

namespace gos {
namespace modbus {

namespace display {
Oled oled;
Two twoline(oled);
bool updated = false;
namespace update {
namespace first {

void line(const char* text) {
  if (gatl::string::compare(gm::format::display::buffer::first, text) == 0) {
    gatl::string::copy(gm::format::display::buffer::first, text);
    updated = true;
  }
}

} // namespace first
namespace second {

void line(const char* text) {
  if (gatl::string::compare(gm::format::display::buffer::second, text) == 0) {
    gatl::string::copy(gm::format::display::buffer::second, text);
    updated = true;
  }
}

void line() {
}

} // namespace second
} // namespace update
} // namespace display

} // namespace modbus
} // namespace gos
