#ifndef GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_BINDING_H_
#define GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_BINDING_H_

#include <gatlbinding.h>

#include "type.h"

#ifndef NO_DISPLAY

namespace gos {
namespace temperature {
namespace binding {

void create();

namespace modbus {

typedef ::gos::atl::binding::reference<type::Unsigned, uint16_t> Unsigned;
typedef ::gos::atl::binding::reference<type::Real, uint16_t> Real;

namespace input {
namespace registers {

extern Unsigned uints;
extern Real real;

} // namespace registers
} // namespace input

namespace holding {
namespace registers {

extern Unsigned uints;
extern Real real;

} // namespace registers
} // namespace holding


} // namespace modbus

} // namespace display
} // namespace temperature
} // namespace gos

#endif
#endif
