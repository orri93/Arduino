#ifndef GOS_ARDUINO_TOOL_SERIAL_H_
#define GOS_ARDUINO_TOOL_SERIAL_H_

#include <string>

namespace gos {
namespace arduino {
namespace tools {
namespace serial {

namespace compensate {
::std::string port(const char* device);
} // namespace compensate

} // namespace serial
} // namespace tools
} // namespace arduino 
} // namespace gos

#endif
