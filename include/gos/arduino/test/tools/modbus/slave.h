#ifndef GOS_ARDUINO_TEST_TOOL_MODBUS_SLAVE_H_
#define GOS_ARDUINO_TEST_TOOL_MODBUS_SLAVE_H_

namespace gos {
namespace arduino {
namespace test {
namespace tools {
namespace modbus {

class slave {
public:
  virtual ~slave() { }
  virtual void begin(const int& baud) = 0;
  virtual void loop() = 0;
};

} // namespace modbus
} // namespace tools
} // namespace test
} // namespace arduino 
} // namespace gos

#endif
