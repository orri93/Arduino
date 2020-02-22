#ifndef GOS_ARDUINO_TEST_TOOL_ARDUINO_H_
#define GOS_ARDUINO_TEST_TOOL_ARDUINO_H_

#include <stdint.h>

#include <gos/arduino/test/tools/arduino/serial.h>

#ifndef ARDUINO_ARCH_AVR
#define ARDUINO_ARCH_AVR
#endif

#define PROGMEM

#define HIGH           0x1
#define LOW            0x0

#define INPUT          0x0
#define OUTPUT         0x1
#define INPUT_PULLUP   0x2


#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif
#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif

#ifndef _HYPER_DEFINED
typedef bool boolean;
typedef uint8_t byte;
#endif

#ifndef bitRead
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#endif
#ifndef bitSet
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#endif
#ifndef bitClear
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#endif
#ifndef bitWrite
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
#endif

void pinMode(uint8_t, uint8_t);
void digitalWrite(uint8_t, uint8_t);
unsigned long millis(void);
unsigned long micros(void);


uint16_t makeWord(uint16_t w);
uint16_t makeWord(byte h, byte l);

#define word(...) makeWord(__VA_ARGS__)


namespace gos {
namespace arduino {
namespace test {
namespace tools {
void initialize();
} // namespace tools
} // namespace test
} // namespace arduino 
} // namespace gos

#endif
