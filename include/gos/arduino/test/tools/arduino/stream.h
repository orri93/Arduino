#ifndef GOS_ARDUINO_TEST_TOOL_ARDUINO_STREAM_H_
#define GOS_ARDUINO_TEST_TOOL_ARDUINO_STREAM_H_

#include <gos/arduino/test/tools/arduino/print.h>

class Stream : public virtual Print {
public:
  Stream();
  virtual ~Stream();

  virtual int available() = 0;
  virtual int read() = 0;

  virtual void setTimeout(unsigned long timeout) = 0;

  virtual size_t readBytes(char* buffer, size_t length) = 0;
  virtual size_t readBytes(uint8_t* buffer, size_t length) = 0;

  virtual size_t write(const uint8_t* buffer, size_t size) = 0;
};

#endif
