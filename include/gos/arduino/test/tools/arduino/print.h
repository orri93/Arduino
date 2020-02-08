#ifndef GOS_ARDUINO_TEST_TOOL_ARDUINO_PRINT_H_
#define GOS_ARDUINO_TEST_TOOL_ARDUINO_PRINT_H_

#include <stdint.h>

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

class Print {
public:
  virtual ~Print();
  virtual size_t write(const uint8_t* buffer, size_t size) = 0;
  virtual int availableForWrite() = 0;
  virtual void flush();
};

#endif
