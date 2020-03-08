#ifndef GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_VARIABLE_H_
#define GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_VARIABLE_H_

#include "type.h"

namespace gos {
namespace temperature {
namespace variables {

extern uint8_t coils;
namespace temporary {
extern type::Unsigned integer;
extern type::Real real;
} // namespace temporary

} // namespace variables
} // namespace temperature
} // namespace gos

#endif
