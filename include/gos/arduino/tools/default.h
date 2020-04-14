#ifndef GOS_ARDUINO_TOOL_DEFAULT_H_
#define GOS_ARDUINO_TOOL_DEFAULT_H_

#include <string>

#include <gos/arduino/tools/types.h>

#ifndef DEFAULT_SERIAL_PORT
#ifdef WIN32
#define DEFAULT_SERIAL_PORT "COM32"
#else
#define DEFAULT_SERIAL_PORT "/dev/ttyS0"
#endif
#endif
#ifndef DEFAULT_BAUD
#define DEFAULT_BAUD 9600
#endif

namespace gos {
namespace arduino {
namespace tools {
namespace default {

  namespace communication {
  namespace serial {
  extern const char* Port;
  extern const int Baud;
  } // namespace serial
  } // namespace communication

  namespace slave {
  extern const int Id;
  } // namespace slave

  namespace timing {
  namespace interval {
  namespace milliseconds {
  extern const int Loop;
  } // namespace milliseconds
  } // namespace interval
  } // namespace timing

} // namespace default
} // namespace tools
} // namespace arduino 
} // namespace gos

#endif
