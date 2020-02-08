#ifndef GOS_ARDUINO_TEST_TOOL_ARDUINO_SERIAL_H_
#define GOS_ARDUINO_TEST_TOOL_ARDUINO_SERIAL_H_

#include <cstdint>

#include <stack>
#include <queue>
#include <memory>
#include <string>

#if defined(_WIN32)
#include <windows.h>
#else
#include <termios.h>
#endif

#include <gos/arduino/test/tools/arduino/stream.h>
#include <gos/arduino/test/tools/types.h>

#if !defined(SERIAL_TX_BUFFER_SIZE)
#if ((RAMEND - RAMSTART) < 1023)
#define SERIAL_TX_BUFFER_SIZE 16
#else
#define SERIAL_TX_BUFFER_SIZE 64
#endif
#endif

#if defined(_WIN32)
#if !defined(ENOTSUP)
#define ENOTSUP WSAEOPNOTSUPP
#endif

/* WIN32: struct containing serial handle and a receive buffer */
#define PY_BUF_SIZE 512
struct win32_ser {
  /* File handle */
  HANDLE fd;
  /* Receive buffer */
  uint8_t buf[PY_BUF_SIZE];
  /* Received chars */
  DWORD n_bytes;
};
#endif /* _WIN32 */

typedef struct _rtu {
  /* Device: "/dev/ttyS0", "/dev/ttyUSB0" or "/dev/tty.USA19*" on Mac OS X. */
  char* device;
  /* Bauds: 9600, 19200, 57600, 115200, etc */
  int baud;
  /* Data bit */
  uint8_t data_bit;
  /* Stop bit */
  uint8_t stop_bit;
  /* Parity: 'N', 'O', 'E' */
  char parity;
#if defined(_WIN32)
  struct win32_ser w_ser;
  DCB old_dcb;
#else
  /* Save old termios settings */
  struct termios old_tios;
#endif
#if HAVE_DECL_TIOCSRS485
  int serial_mode;
#endif
#if HAVE_DECL_TIOCM_RTS
  int rts;
  int rts_delay;
  int onebyte_time;
  void (*set_rts) (modbus_t* ctx, int on);
#endif
  /* To handle many slaves on the same link */
  int confirmation_to_ignore;
} rtu_t;

class _Serial : public Stream {
public:
  _Serial();
  _Serial(
    const std::string& device,
    const int& baud,
    const int& databits,
    const int& stopbits,
    const char& parity);
  virtual ~_Serial();

  bool initialize();
  void shutdown();

  void loop();

  /* Stream implementation */
  int available();
  int read();

  /* Sets the maximum milliseconds to wait for serial data.
     It defaults to 1000 milliseconds. */
  void setTimeout(unsigned long timeout);

  size_t readBytes(char* buffer, size_t length);
  size_t readBytes(uint8_t* buffer, size_t length);

  /* Print implementation  */
  size_t write(const uint8_t* buffer, size_t size);
  /* Get the number of bytes (characters) available for writing in
     the serial buffer without blocking the write operation. */
  int availableForWrite();

private:
  typedef std::unique_ptr<rtu_t> RtuPointer;
  typedef std::stack<uint8_t> Stack;
  typedef std::queue<uint8_t> Queue;

  int size();
  bool empty();
  void push(const uint8_t value);
  uint8_t pop();

  RtuPointer rtu_;
  ::gos::arduino::test::tools::types::TextPointer device_;
  timeval tv_;

  Stack stack_;
  Queue queue_;
};

extern _Serial Serial;

#endif
