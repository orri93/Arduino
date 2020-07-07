#ifndef GOS_ARDUINO_TOOL_TYPES_H_
#define GOS_ARDUINO_TOOL_TYPES_H_

#include <cstdint>

#include <chrono>
#include <string>
#include <utility>

#ifdef NOT_USED
#undef NOT_USED
#endif

namespace gos {
namespace arduino {
namespace tools {
namespace types {

enum class level {
  silent,
  normal,
  verbose
};

#ifdef GOS_NOT_YET_USED
namespace status {
enum class bus { undefined, idle, active, connected };
int from(const ::nov::dlink::bus::type::status::work& state);
::nov::dlink::bus::type::status::work to(const int& number);
std::string from(const bus& state, const bool& firstcapital = false);
}
#endif

typedef ::std::pair<::std::string, uint16_t> endpoint;
endpoint make_endpoint(const ::std::string& address, const uint16_t& port);

bool iszero(const endpoint& endpoint);
bool isempty(const endpoint& endpoint);

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

} // namespace types
} // namespace tools
} // namespace arduino 
} // namespace gos
#endif
