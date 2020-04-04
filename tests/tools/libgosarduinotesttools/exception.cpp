#include <strstream>

#include <gos/arduino/test/tools/exception.h>

namespace gos {
namespace arduino {
namespace test {
namespace tools {
exception::exception(const char* what) {
  std::strstream s;
  s << "Arduino test tool error: " << what << std::ends;
  what_ = s.str();
}
#if _MSC_VER >= 1910
const char* exception::what() const noexcept { return what_.c_str(); }
#else
const char* exception::what() const { return what_.c_str(); }
#endif
} // namespace tools
} // namespace test
} // namespace arduino 
} // namespace gos
