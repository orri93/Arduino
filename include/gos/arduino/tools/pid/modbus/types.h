#ifndef GOS_ARDUINO_TOOL_PID_MODBUS_TYPES_H_
#define GOS_ARDUINO_TOOL_PID_MODBUS_TYPES_H_

namespace gos {
namespace arduino {
namespace tools {
namespace pid {
namespace modbus {
namespace types {

enum class result {
  undefiend,
  success,
  failure,
  fatal,
  uninitialized,
  disconnected
};

} // namespace types
} // namespace modbus
} // namespace pid
} // namespace tools
} // namespace arduino 
} // namespace gos

#endif
