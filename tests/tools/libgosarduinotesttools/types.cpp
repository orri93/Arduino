#include <utility>

#include <gos/arduino/test/tools/types.h>

namespace gos {
namespace arduino {
namespace test {
namespace tools {
namespace types {

endpoint make_endpoint(
  const ::std::string& address,
  const uint16_t& port) {
  return ::std::make_pair(address, port);
}

bool iszero(const endpoint& endpoint) {
  return endpoint.first.empty() == 0 && endpoint.second == 0;
}
bool isempty(const endpoint& endpoint) {
  return endpoint.first.empty() && endpoint.second == 0;
}

#ifdef GOS_NOT_YET_USED
namespace status {
int from(const ndb::type::status::work& state) {
  switch (state) {
  case ndb::type::status::work::undefined:
    return -1;
  case ndb::type::status::work::unbegun:
    return 0;
  case ndb::type::status::work::current:
    return 1;
  case ndb::type::status::work::completed:
    return 1;
  default:
    return -2;
  }
}
ndb::type::status::work to(const int& number) {
  switch (number) {
  case 0:
    return ndb::type::status::work::unbegun;
  case 1:
    return ndb::type::status::work::current;
  case 2:
    return ndb::type::status::work::completed;
  default:
    return ndb::type::status::work::undefined;
  }
}
std::string ndb::type::status::from(const bus& state, const bool& firstcapital) {
  switch (state) {
  case ndb::type::status::bus::undefined:
    return firstcapital ? "Undefined" : "undefined";
  case ndb::type::status::bus::idle:
    return firstcapital ? "Idle" : "idle";
  case ndb::type::status::bus::active:
    return firstcapital ? "Active" : "active";
  case ndb::type::status::bus::connected:
    return firstcapital ? "Connected" : "connected";
  default:
    return firstcapital ? "Unknown" : "unknown";
  }
}
}
#endif

} // namespace types
} // namespace tools
} // namespace test
} // namespace arduino 
} // namespace gos
