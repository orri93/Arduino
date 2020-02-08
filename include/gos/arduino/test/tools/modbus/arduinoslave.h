#ifndef GOS_ARDUINO_TEST_TOOL_MODBUS_ARDUINO_SLAVE_H_
#define GOS_ARDUINO_TEST_TOOL_MODBUS_ARDUINO_SLAVE_H_

#include <gos/arduino/test/tools/modbus/slave.h>
#include <gos/arduino/test/tools/arduino/stream.h>

#include <ModbusSlave.h>

namespace gos {
namespace arduino {
namespace test {
namespace tools {
namespace modbus {

class modbusslave : public virtual slave {
public:
  modbusslave(Stream& stream, const int& id);
  void begin(const int& baud);
  void loop();
};

} // namespace modbus
} // namespace tools
} // namespace test
} // namespace arduino 
} // namespace gos

#endif
