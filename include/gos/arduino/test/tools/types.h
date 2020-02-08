#ifndef GOS_ARDUINO_TEST_TOOL_TYPES_H_
#define GOS_ARDUINO_TEST_TOOL_TYPES_H_

/* For uint16_t        */
#include <cstdint>

/* For std::string     */
#include <string>
/* For std::pair       */
#include <utility>
/* For std::unique_ptr */
#include <memory>

#include <chrono>


#ifdef NOT_USED
#undef NOT_USED
#endif

namespace gos {
namespace arduino {
namespace test {
namespace tools {
namespace types {

typedef std::unique_ptr<char[]> TextPointer;

enum class level {
  silent,
  normal,
  verbose
};

namespace steady {
typedef std::chrono::steady_clock Clock;
typedef Clock::duration Duration;
typedef Clock::time_point Time;
}

namespace high {
namespace resolution {
typedef std::chrono::high_resolution_clock Clock;
typedef Clock::duration Duration;
typedef Clock::time_point Time;
}
}

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

} // namespace types
} // namespace tools
} // namespace test
} // namespace arduino 
} // namespace gos

#endif
