#ifndef GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_VARIABLE_H_
#define GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_VARIABLE_H_

#include "type.h"

#define GOS_TCV_COIL_BIT_PONE               0
#define GOS_TCV_COIL_BIT_TUNE_TIME_MASTER   1

#define GOT_PI_TUNE_TIME_UNIT_DEFAULT       0
#define GOT_PI_TUNE_TIME_UNIT_MILLISECONDS  1
#define GOT_PI_TUNE_TIME_UNIT_SECONDS       2
#define GOT_PI_TUNE_TIME_UNIT_MINUTES       3
#define GOT_PI_TUNE_TIME_UNIT_MAXIMUM       3

#define GOT_PI_TUNE_TIME_FORCE_OFF          0
#define GOT_PI_TUNE_TIME_FORCE_IDLE         1
#define GOT_PI_TUNE_TIME_FORCE_MANUAL       2
#define GOT_PI_TUNE_TIME_FORCE_AUTO         3
#define GOT_PI_TUNE_TIME_FORCE_MAXIMUM      3

namespace gos {
namespace temperature {
namespace variables {

extern type::Status status;

extern type::Unsigned force;

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

namespace pid {
namespace tune {
namespace time {
extern type::Unsigned unit;
}
}
}

namespace temporary {
extern bool boolean;
extern uint8_t byte;
extern type::Unsigned integer;
extern type::Real real;
} // namespace temporary

} // namespace variables
} // namespace temperature
} // namespace gos

#endif
