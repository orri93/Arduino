#ifndef GOS_ARDUINO_TEST_TOOL_DEFAULT_H_
#define GOS_ARDUINO_TEST_TOOL_DEFAULT_H_

#include <string>

#include <gos/arduino/test/tools/types.h>

#ifndef DEFAULT_SERIAL_PORT
#ifdef WIN32
#define DEFAULT_SERIAL_PORT "COM1"
#else
#define DEFAULT_SERIAL_PORT "/dev/ttyS0"
#endif
#endif
#ifndef DEFAULT_BAUD
#define DEFAULT_BAUD 9600
#endif
#ifndef DEFAULT_DATA_BITS
#define DEFAULT_DATA_BITS 8
#endif
#ifndef DEFAULT_STOP_BITS
#define DEFAULT_STOP_BITS 1
#endif
#ifndef DEFAULT_PARITY
#define DEFAULT_PARITY 'N'
#endif

namespace gos {
namespace arduino {
namespace test {
namespace tools {
namespace default {

extern ::gos::arduino::test::tools::types::level Verbosity;

namespace communication {
namespace serial {
extern const char* Port;
extern const int Baud;
namespace data {
extern const int Bits;
} // namespace data
namespace stop {
extern const int Bits;
} // namespace stop
extern const char Parity;
} // namespace serial
} // namespace communication

namespace timing {
namespace interval {
namespace milliseconds {
extern const int Loop;
} // namespace milliseconds
} // namespace interval
} // namespace timing

} // namespace default
} // namespace tools
} // namespace test
} // namespace arduino 
} // namespace gos

#endif
