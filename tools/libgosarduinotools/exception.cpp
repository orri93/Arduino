#include <strstream>

#include <gos/arduino/tools/exception.h>

namespace gos {
namespace arduino {
namespace tools {
exception::exception(const char* what) {
  std::strstream s;
  s << "DrillLink tool error: " << what << std::ends;
  _what = s.str();
}
#if _MSC_VER >= 1910
const char* exception::what() const noexcept { return _what.c_str(); }
#else
const char* exception::what() const { return _what.c_str(); }
#endif
}
}
}
