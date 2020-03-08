#include "display.h"

#ifndef NO_DISPLAY

namespace gatl = ::gos::atl;

namespace gos {
namespace temperature {
namespace display {

Oled oled;
Two two(oled);

} // namespace display
} // namespace temperature
} // namespace gos

#endif
