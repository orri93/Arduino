#ifndef GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_VARIABLE_H_
#define GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_VARIABLE_H_

#include "type.h"

#define GOS_TCV_COIL_BIT_PONE             0
#define GOS_TCV_COIL_BIT_TUNE_TIME_MASTER 1

namespace gos {
namespace temperature {
namespace variables {

extern type::Unsigned output;
extern type::Real temperature;

namespace timing {
extern unsigned long tick;
extern unsigned long next;
extern type::Unsigned interval;
}

namespace controller {
extern type::Unsigned manual;
}

namespace modbus {
extern uint8_t coils;
} // namespace modbus

namespace temporary {
extern bool boolean;
extern type::Unsigned integer;
extern type::Real real;
} // namespace temporary

} // namespace variables
} // namespace temperature
} // namespace gos

#endif
