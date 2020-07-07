#ifndef GOS_ARDUINO_TOOL_UTILITIES_H_
#define GOS_ARDUINO_TOOL_UTILITIES_H_

#include <gos/arduino/tools/types.h>

namespace gos {
namespace arduino {
namespace tools {
namespace utilities {

template<typename T>
T scale(const ::gos::arduino::tools::types::range<T>& range) {
  return range.highest - range.lowest;
}

template<typename T>
T midpoint(const ::gos::arduino::tools::types::range<T>& range) {
  return range.lowest + scale(range) / 2;
}

template<typename T>
T restrict(
  const ::gos::arduino::tools::types::range<T>& range,
  const T& value) {
  if (value <= range.highest && value >= range.lowest) {
    return value;
  } else if (value > range.highest) {
    return range.highest;
  } else {
    return range.lowest;
  }
}

} // namespace utilities
} // namespace tools
} // namespace arduino 
} // namespace gos
#endif
