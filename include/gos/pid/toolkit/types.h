#ifndef GOS_PID_TOOLKIT_TYPES_H_
#define GOS_PID_TOOLKIT_TYPES_H_

#include <string>
#include <exception>

namespace gos {
namespace pid {
namespace toolkit {
namespace type {

template<typename T> struct range {
  T lowest;
  T highest;
};

template<typename T> range<T> make_range(const T& lowest, const T& highest) {
  range<T> range;
  range.lowest = lowest;
  range.highest = highest;
  return range;
}

} // namespace type
} // namespace toolkit
} // namespace pid
} // namespace gos

#endif
