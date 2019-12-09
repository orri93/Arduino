/* To get rid of C4005 warning when modbus-tcp.h defines some error macros */

#include <gos/arduino/tools/default.h>

namespace gos {
namespace arduino {
namespace tools {
namespace default {
const int Interval = 0;

namespace slave {
const int Id = MODBUS_DEFAULT_SLAVE_ID;
}
namespace serial {
const int Baud = MODBUS_RTU_DEFAULT_BAUD;
const char* Port = MODBUS_RTU_DEFAULT_SERIAL_PORT;
namespace string {
const std::string Port = ::gos::arduino::tools::default::serial::Port;
}
}

}
}
}
}
