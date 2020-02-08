/* To get rid of C4005 warning when modbus-tcp.h defines some error macros */

#include <gos/arduino/test/tools/default.h>

namespace gatt = ::gos::arduino::test::tools;

namespace gos {
namespace arduino {
namespace test {
namespace tools {
namespace default {

gatt::types::level Verbosity = gatt::types::level::normal;

namespace communication {
namespace serial {
const char* Port = DEFAULT_SERIAL_PORT;
const int Baud = DEFAULT_BAUD;
namespace data {
const int Bits = DEFAULT_DATA_BITS;
} // namespace data
namespace stop {
const int Bits = DEFAULT_STOP_BITS;
} // namespace stop
const char Parity = DEFAULT_PARITY;
} // namespace serial
} // namespace communication

namespace timing {
namespace interval {
namespace milliseconds {
const int Loop = 0;
} // namespace milliseconds
} // namespace interval
} // namespace timing

} // namespace default
} // namespace tools
} // namespace test
} // namespace arduino 
} // namespace gos
