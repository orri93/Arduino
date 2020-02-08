#include <iostream>

#include <gos/arduino/test/tools/arduino/Arduino.h>
#include <gos/arduino/test/tools/types.h>

namespace gatt = ::gos::arduino::test::tools;
namespace gattt = ::gos::arduino::test::tools::types;

namespace gos {
namespace arduino {
namespace test {
namespace tools {

typedef gattt::steady::Clock Clock;
typedef gattt::steady::Duration Duration;
typedef gattt::steady::Time Time;

static Time _time;

void initialize() {
  _time = Clock::now();
}

} // namespace tools
} // namespace test
} // namespace arduino 
} // namespace gos

void pinMode(uint8_t, uint8_t) {
  /* Do nothing */
}

void digitalWrite(uint8_t pin, uint8_t value) {
  std::cout << "Setting pin "
    << static_cast<int>(pin) << " to "
    << static_cast<int>(value) << std::endl;
}

unsigned long millis(void) {
  gatt::Duration duration = gatt::Clock::now() - gatt::_time;
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration);
  return static_cast<unsigned long>(ms.count());
}

unsigned long micros(void) {
  gatt::Duration duration = gatt::Clock::now() - gatt::_time;
  auto ms = std::chrono::duration_cast<std::chrono::microseconds>(duration);
  return static_cast<unsigned long>(ms.count());
}

uint16_t makeWord(uint16_t w) {
  return w;
}

uint16_t makeWord(byte h, byte l) {
  return static_cast<uint16_t>(h) << 8 | static_cast<uint16_t>(l);
}
