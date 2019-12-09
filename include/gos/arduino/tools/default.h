#ifndef GOS_ARDUINO_TOOL_DEFAULT_H_
#define GOS_ARDUINO_TOOL_DEFAULT_H_

#include <string>

#define MODBUS_DEFAULT_SLAVE_ID 1
#define MODBUS_RTU_DEFAULT_SERIAL_PORT "COM11"
#define MODBUS_RTU_DEFAULT_BAUD 9600

namespace gos {
namespace arduino {
namespace tools {
namespace default {

extern const int Interval;

namespace slave {
extern const int Id;
}
namespace serial {
extern const int Baud;
extern const char* Port;
namespace string {
extern const std::string Port;
}
}

}
}
}
}

#endif
