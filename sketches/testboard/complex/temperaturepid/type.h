#ifndef GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_TYPE_H_
#define GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_TYPE_H_

#include <Arduino.h>

namespace gos {
namespace temperature {

namespace type {
#ifdef USE_FLOAT_FOR_REAL
typedef float Real;
#else
typedef double Real;
#endif
typedef int16_t Signed;
typedef uint16_t Unsigned;
} // namespace type

} // namespace temperature
} // namespace gos

#endif
