#include <regex>

namespace gos {
namespace arduino {
namespace tools {
namespace serial {

namespace compensate {
::std::string port(const char* device) {
  std::string port(device);
#ifdef _WIN32
  std::regex highport("COM\\d{2}");
  if (std::regex_match(port, highport)) {
    port = "\\\\.\\" + port;
  }
#endif
  return port;
}
} // namespace compensate

} // namespace serial
} // namespace tools
} // namespace arduino 
} // namespace gos
