#include <cstdio>

#include <iostream>

#include <gos/arduino/test/tools/arduino/serial.h>

#define GOS_ARDUINO_TEST_TOOL_ARDUINO_SERIAL_BUFFER_SIZE 1024

#define GOS_WIN32_SERIAL_LENGTH_TO_READ \
  GOS_ARDUINO_TEST_TOOL_ARDUINO_SERIAL_BUFFER_SIZE

#define USE_SELECT

const size_t BufferSize = GOS_ARDUINO_TEST_TOOL_ARDUINO_SERIAL_BUFFER_SIZE;

static uint8_t _buffer[BufferSize];

static void win32_ser_init(struct win32_ser* ws);

#if defined(_WIN32)

/* This simple implementation is sort of a substitute of the select() call,
 * working this way: the win32_ser_select() call tries to read some data from
 * the serial port, setting the timeout as the select() call would. Data read is
 * stored into the receive buffer, that is then consumed by the win32_ser_read()
 * call.  So win32_ser_select() does both the event waiting and the reading,
 * while win32_ser_read() only consumes the receive buffer.
 */

static void win32_ser_init(struct win32_ser* ws);

/* FIXME Try to remove length_to_read -> max_len argument, only used by win32 */
static int win32_ser_select(
  struct win32_ser* ws,
  int max_len,
  const struct timeval* tv);
static int win32_ser_read(
  struct win32_ser* ws,
  uint8_t* p_msg,
  unsigned int max_len);
#endif

#if HAVE_DECL_TIOCM_RTS
static void _modbus_rtu_ioctl_rts(modbus_t* ctx, int on);
#endif

static size_t _rtu_send(rtu_t& rtu, const uint8_t* req, int req_length);
static size_t _rtu_recv(rtu_t& rtu, uint8_t* rsp, int rsp_length);
static int _rtu_connect(rtu_t& rtu);

static int rtu_set_serial_mode(rtu_t& rtu, int mode);
static int rtu_get_serial_mode(rtu_t& rtu);
static int rtu_get_rts(rtu_t& rtu);
static int rtu_set_rts(rtu_t& rtu, int mode);
static int rtu_set_custom_rts(rtu_t& rtu, void (*set_rts) (rtu_t& rtu, int on));
static int rtu_get_rts_delay(rtu_t& rtu);
static int rtu_set_rts_delay(rtu_t& rtu, int us);

static void _rtu_close(rtu_t& rtu);

static int _rtu_flush(rtu_t& rtu);

static int _rtu_select(
  rtu_t& rtu,
  fd_set* rset,
  struct timeval* tv,
  int length_to_read);

static bool new_rtu(rtu_t& rtu);

_Serial Serial;

_Serial::_Serial() : tv_{ 0, 0 } {
}

_Serial::_Serial(
  const std::string& device,
  const int& baud,
  const int& databits,
  const int& stopbits,
  const char& parity) : tv_{ 0, 0 } {
  device_ = std::make_unique<char[]>(device.length() + 1);
  if (device_) {
    ::memcpy(device_.get(), device.c_str(), device.length() + 1);
    rtu_ = std::make_unique<rtu_t>();
    if (rtu_) {
      rtu_->device = device_.get();
      rtu_->baud = baud;
      rtu_->data_bit = static_cast<uint8_t>(databits);
      rtu_->stop_bit = static_cast<uint8_t>(stopbits);
      rtu_->parity = parity;
    }
  }
}

_Serial ::~_Serial() {
}

bool _Serial::initialize() {
  if (rtu_) {
    if (!::new_rtu(*rtu_)) {
      return false;
    }
    if (::_rtu_connect(*rtu_) < 0) {
      return false;
    }
  } else {
    return false;
  }
  return true;
}

void _Serial::shutdown() {
  if (rtu_) {
    _rtu_flush(*rtu_);
    _rtu_close(*rtu_);
  }
}

void _Serial::loop() {
  if (rtu_) {
    size_t received;
#ifdef USE_SELECT
    int select;
    do {
      select = _rtu_select(*rtu_, nullptr, &tv_, BufferSize);
      if (select > 0) {
        received = _rtu_recv(*rtu_, _buffer, BufferSize);
        if (received > 0) {
          for (int i = 0; i < received; ++i) {
            push(_buffer[i]);
          }
        }
      }
    } while (select > 0);
#else
    do {
      received = _rtu_recv(rtu_, _buffer, BufferSize);
      if (received > 0) {
        for (int i = 0; i < received; ++i) {
          push(_buffer[i]);
        }
      }
    } while (received > 0);
#endif
  }
}

/* Stream implementation */
int _Serial::available() {
  int available = static_cast<int>(size());
  if (available > 0) {
    std::cout << "Serial available is " << available << std::endl;
  }
  return available;
}

int _Serial::read() {
  if (!empty()) {
    return static_cast<int>(pop());
  } else {
    return -1;
  }
}

void _Serial::setTimeout(unsigned long timeout) {
  std::cout << "Serial timeout set to " << timeout << std::endl;
  uint64_t ms = 1000 * static_cast<uint64_t>(timeout);
  tv_.tv_sec = static_cast<long>(ms / 1000000);
  tv_.tv_usec = static_cast<long>(
    ms - 1000000 * static_cast<uint64_t>(tv_.tv_sec));
}

size_t _Serial::readBytes(char* buffer, size_t length) {
  size_t result = 0;
  while (!empty() && length > 0) {
    *buffer = static_cast<char>(pop());
    ++buffer;
    ++result;
    --length;
  }
  return result;
}

size_t _Serial::readBytes(uint8_t* buffer, size_t length) {
  size_t result = 0;
  while (!empty() && length > 0) {
    *buffer = pop();
    ++buffer;
    ++result;
    --length;
  }
  return result;
}

/* Print implementation  */
size_t _Serial::write(const uint8_t* buffer, size_t size) {
  if (rtu_) {
    std::cout << "Serial last write wrote " << size << " bytes" << std::endl;
    return _rtu_send(*rtu_, buffer, static_cast<int>(size));
  } else {
    return 0;
  }
}

int _Serial::availableForWrite() {
  return 0;
}

int _Serial::size() {
  return static_cast<int>(queue_.size());
}

bool _Serial::empty() {
  return queue_.empty();
}

void _Serial::push(const uint8_t value) {
  queue_.push(value);
}

uint8_t _Serial::pop() {
  uint8_t result = queue_.front();
  queue_.pop();
  return result;
}

#if defined(_WIN32)
/* This simple implementation is sort of a substitute of the select() call,
 * working this way: the win32_ser_select() call tries to read some data from
 * the serial port, setting the timeout as the select() call would. Data read is
 * stored into the receive buffer, that is then consumed by the win32_ser_read()
 * call.  So win32_ser_select() does both the event waiting and the reading,
 * while win32_ser_read() only consumes the receive buffer.
 */

void win32_ser_init(struct win32_ser* ws) {
  /* Clear everything */
  memset(ws, 0x00, sizeof(struct win32_ser));

  /* Set file handle to invalid */
  ws->fd = INVALID_HANDLE_VALUE;
}

/* FIXME Try to remove length_to_read -> max_len argument, only used by win32 */
int win32_ser_select(
  struct win32_ser* ws,
  int max_len,
  const struct timeval* tv) {
  COMMTIMEOUTS comm_to;
  unsigned int msec = 0;

  /* Check if some data still in the buffer to be consumed */
  if (ws->n_bytes > 0) {
    return 1;
  }

  /* Setup timeouts like select() would do.
     FIXME Please someone on Windows can look at this?
     Does it possible to use WaitCommEvent?
     When tv is NULL, MAXDWORD isn't infinite!
   */
  if (tv == NULL) {
    msec = MAXDWORD;
  } else {
    msec = tv->tv_sec * 1000 + tv->tv_usec / 1000;
    if (msec < 1)
      msec = 1;
  }

  comm_to.ReadIntervalTimeout = msec;
  comm_to.ReadTotalTimeoutMultiplier = 0;
  comm_to.ReadTotalTimeoutConstant = msec;
  comm_to.WriteTotalTimeoutMultiplier = 0;
  comm_to.WriteTotalTimeoutConstant = 1000;
  SetCommTimeouts(ws->fd, &comm_to);

  /* Read some bytes */
  if ((max_len > PY_BUF_SIZE) || (max_len < 0)) {
    max_len = PY_BUF_SIZE;
  }

  if (ReadFile(ws->fd, &ws->buf, max_len, &ws->n_bytes, NULL)) {
    /* Check if some bytes available */
    if (ws->n_bytes > 0) {
      /* Some bytes read */
      return 1;
    } else {
      /* Just timed out */
      return 0;
    }
  } else {
    /* Some kind of error */
    return -1;
  }
}

int win32_ser_read(struct win32_ser* ws, uint8_t* p_msg, unsigned int max_len) {
  unsigned int n = ws->n_bytes;

  if (max_len < n) {
    n = max_len;
  }

  if (n > 0) {
    memcpy(p_msg, ws->buf, n);
  }

  ws->n_bytes -= n;

  return n;
}
#endif

#if HAVE_DECL_TIOCM_RTS
void _modbus_rtu_ioctl_rts(modbus_t* ctx, int on) {
  int fd = ctx->s;
  int flags;

  ioctl(fd, TIOCMGET, &flags);
  if (on) {
    flags |= TIOCM_RTS;
  } else {
    flags &= ~TIOCM_RTS;
  }
  ioctl(fd, TIOCMSET, &flags);
}
#endif

size_t _rtu_send(rtu_t& rtu, const uint8_t* req, int req_length) {
#if defined(_WIN32)
  DWORD n_bytes = 0;
  return (WriteFile(rtu.w_ser.fd, req, req_length, &n_bytes, NULL)) ?
    (size_t)n_bytes : -1;
#else
#if HAVE_DECL_TIOCM_RTS
  modbus_rtu_t* ctx_rtu = ctx->backend_data;
  if (ctx_rtu->rts != MODBUS_RTU_RTS_NONE) {
    ssize_t size;

    if (ctx->debug) {
      fprintf(stderr, "Sending request using RTS signal\n");
    }

    ctx_rtu->set_rts(ctx, ctx_rtu->rts == MODBUS_RTU_RTS_UP);
    usleep(ctx_rtu->rts_delay);

    size = write(ctx->s, req, req_length);

    usleep(ctx_rtu->onebyte_time * req_length + ctx_rtu->rts_delay);
    ctx_rtu->set_rts(ctx, ctx_rtu->rts != MODBUS_RTU_RTS_UP);

    return size;
  } else {
#endif
    return write(ctx->s, req, req_length);
#if HAVE_DECL_TIOCM_RTS
  }
#endif
#endif
}

size_t _rtu_recv(rtu_t& rtu, uint8_t* rsp, int rsp_length) {
#if defined(_WIN32)
  return win32_ser_read(&(rtu.w_ser), rsp, rsp_length);
#else
  return read(ctx->s, rsp, rsp_length);
#endif
}

/* Sets up a serial port for RTU communications */
int _rtu_connect(rtu_t& rtu) {
#if defined(_WIN32)
  DCB dcb;
#else
  struct termios tios;
  speed_t speed;
  int flags;
#endif

  printf("Opening %s at %d bauds (%c, %d, %d)\n",
    rtu.device, rtu.baud, rtu.parity, rtu.data_bit, rtu.stop_bit);

#if defined(_WIN32)
  /* Some references here:
   * http://msdn.microsoft.com/en-us/library/aa450602.aspx
   */
  win32_ser_init(&rtu.w_ser);

  /* ctx_rtu->device should contain a string like "COMxx:" xx being a decimal
   * number */
  rtu.w_ser.fd = CreateFileA(rtu.device,
    GENERIC_READ | GENERIC_WRITE,
    0,
    NULL,
    OPEN_EXISTING,
    0,
    NULL);

  /* Error checking */
  if (rtu.w_ser.fd == INVALID_HANDLE_VALUE) {
    fprintf(stderr, "ERROR Can't open the device %s (LastError %d)\n",
      rtu.device, (int)GetLastError());
    return -1;
  }

  /* Save params */
  rtu.old_dcb.DCBlength = sizeof(DCB);
  if (!GetCommState(rtu.w_ser.fd, &(rtu.old_dcb))) {
    fprintf(stderr, "ERROR Error getting configuration (LastError %d)\n",
      (int)GetLastError());
    CloseHandle(rtu.w_ser.fd);
    rtu.w_ser.fd = INVALID_HANDLE_VALUE;
    return -1;
  }

  /* Build new configuration (starting from current settings) */
  dcb = rtu.old_dcb;

  /* Speed setting */
  switch (rtu.baud) {
  case 110:
    dcb.BaudRate = CBR_110;
    break;
  case 300:
    dcb.BaudRate = CBR_300;
    break;
  case 600:
    dcb.BaudRate = CBR_600;
    break;
  case 1200:
    dcb.BaudRate = CBR_1200;
    break;
  case 2400:
    dcb.BaudRate = CBR_2400;
    break;
  case 4800:
    dcb.BaudRate = CBR_4800;
    break;
  case 9600:
    dcb.BaudRate = CBR_9600;
    break;
  case 14400:
    dcb.BaudRate = CBR_14400;
    break;
  case 19200:
    dcb.BaudRate = CBR_19200;
    break;
  case 38400:
    dcb.BaudRate = CBR_38400;
    break;
  case 57600:
    dcb.BaudRate = CBR_57600;
    break;
  case 115200:
    dcb.BaudRate = CBR_115200;
    break;
  case 230400:
    /* CBR_230400 - not defined */
    dcb.BaudRate = 230400;
    break;
  case 250000:
    dcb.BaudRate = 250000;
    break;
  case 460800:
    dcb.BaudRate = 460800;
    break;
  case 500000:
    dcb.BaudRate = 500000;
    break;
  case 921600:
    dcb.BaudRate = 921600;
    break;
  case 1000000:
    dcb.BaudRate = 1000000;
    break;
  default:
    dcb.BaudRate = CBR_9600;
    fprintf(stderr, "WARNING Unknown baud rate %d for %s (B9600 used)\n",
      rtu.baud, rtu.device);
  }

  /* Data bits */
  switch (rtu.data_bit) {
  case 5:
    dcb.ByteSize = 5;
    break;
  case 6:
    dcb.ByteSize = 6;
    break;
  case 7:
    dcb.ByteSize = 7;
    break;
  case 8:
  default:
    dcb.ByteSize = 8;
    break;
  }

  /* Stop bits */
  if (rtu.stop_bit == 1)
    dcb.StopBits = ONESTOPBIT;
  else /* 2 */
    dcb.StopBits = TWOSTOPBITS;

  /* Parity */
  if (rtu.parity == 'N') {
    dcb.Parity = NOPARITY;
    dcb.fParity = FALSE;
  } else if (rtu.parity == 'E') {
    dcb.Parity = EVENPARITY;
    dcb.fParity = TRUE;
  } else {
    /* odd */
    dcb.Parity = ODDPARITY;
    dcb.fParity = TRUE;
  }

  /* Hardware handshaking left as default settings retrieved */

  /* No software handshaking */
  dcb.fTXContinueOnXoff = TRUE;
  dcb.fOutX = FALSE;
  dcb.fInX = FALSE;

  /* Binary mode (it's the only supported on Windows anyway) */
  dcb.fBinary = TRUE;

  /* Don't want errors to be blocking */
  dcb.fAbortOnError = FALSE;

  /* Setup port */
  if (!SetCommState(rtu.w_ser.fd, &dcb)) {
    fprintf(stderr, "ERROR Error setting new configuration (LastError %d)\n",
      (int)GetLastError());
    CloseHandle(rtu.w_ser.fd);
    rtu.w_ser.fd = INVALID_HANDLE_VALUE;
    return -1;
  }
#else
  /* The O_NOCTTY flag tells UNIX that this program doesn't want
     to be the "controlling terminal" for that port. If you
     don't specify this then any input (such as keyboard abort
     signals and so forth) will affect your process

     Timeouts are ignored in canonical input mode or when the
     NDELAY option is set on the file via open or fcntl */
  flags = O_RDWR | O_NOCTTY | O_NDELAY | O_EXCL;
#ifdef O_CLOEXEC
  flags |= O_CLOEXEC;
#endif

  ctx->s = open(ctx_rtu->device, flags);
  if (ctx->s == -1) {
    if (ctx->debug) {
      fprintf(stderr, "ERROR Can't open the device %s (%s)\n",
        ctx_rtu->device, strerror(errno));
    }
    return -1;
  }

  /* Save */
  tcgetattr(ctx->s, &ctx_rtu->old_tios);

  memset(&tios, 0, sizeof(struct termios));

  /* C_ISPEED     Input baud (new interface)
     C_OSPEED     Output baud (new interface)
  */
  switch (ctx_rtu->baud) {
  case 110:
    speed = B110;
    break;
  case 300:
    speed = B300;
    break;
  case 600:
    speed = B600;
    break;
  case 1200:
    speed = B1200;
    break;
  case 2400:
    speed = B2400;
    break;
  case 4800:
    speed = B4800;
    break;
  case 9600:
    speed = B9600;
    break;
  case 19200:
    speed = B19200;
    break;
  case 38400:
    speed = B38400;
    break;
#ifdef B57600
  case 57600:
    speed = B57600;
    break;
#endif
#ifdef B115200
  case 115200:
    speed = B115200;
    break;
#endif
#ifdef B230400
  case 230400:
    speed = B230400;
    break;
#endif
#ifdef B460800
  case 460800:
    speed = B460800;
    break;
#endif
#ifdef B500000
  case 500000:
    speed = B500000;
    break;
#endif
#ifdef B576000
  case 576000:
    speed = B576000;
    break;
#endif
#ifdef B921600
  case 921600:
    speed = B921600;
    break;
#endif
#ifdef B1000000
  case 1000000:
    speed = B1000000;
    break;
#endif
#ifdef B1152000
  case 1152000:
    speed = B1152000;
    break;
#endif
#ifdef B1500000
  case 1500000:
    speed = B1500000;
    break;
#endif
#ifdef B2500000
  case 2500000:
    speed = B2500000;
    break;
#endif
#ifdef B3000000
  case 3000000:
    speed = B3000000;
    break;
#endif
#ifdef B3500000
  case 3500000:
    speed = B3500000;
    break;
#endif
#ifdef B4000000
  case 4000000:
    speed = B4000000;
    break;
#endif
  default:
    speed = B9600;
    if (ctx->debug) {
      fprintf(stderr,
        "WARNING Unknown baud rate %d for %s (B9600 used)\n",
        ctx_rtu->baud, ctx_rtu->device);
    }
  }

  /* Set the baud rate */
  if ((cfsetispeed(&tios, speed) < 0) ||
    (cfsetospeed(&tios, speed) < 0)) {
    close(ctx->s);
    ctx->s = -1;
    return -1;
  }

  /* C_CFLAG      Control options
     CLOCAL       Local line - do not change "owner" of port
     CREAD        Enable receiver
  */
  tios.c_cflag |= (CREAD | CLOCAL);
  /* CSIZE, HUPCL, CRTSCTS (hardware flow control) */

  /* Set data bits (5, 6, 7, 8 bits)
     CSIZE        Bit mask for data bits
  */
  tios.c_cflag &= ~CSIZE;
  switch (ctx_rtu->data_bit) {
  case 5:
    tios.c_cflag |= CS5;
    break;
  case 6:
    tios.c_cflag |= CS6;
    break;
  case 7:
    tios.c_cflag |= CS7;
    break;
  case 8:
  default:
    tios.c_cflag |= CS8;
    break;
  }

  /* Stop bit (1 or 2) */
  if (ctx_rtu->stop_bit == 1)
    tios.c_cflag &= ~CSTOPB;
  else /* 2 */
    tios.c_cflag |= CSTOPB;

  /* PARENB       Enable parity bit
     PARODD       Use odd parity instead of even */
  if (ctx_rtu->parity == 'N') {
    /* None */
    tios.c_cflag &= ~PARENB;
  } else if (ctx_rtu->parity == 'E') {
    /* Even */
    tios.c_cflag |= PARENB;
    tios.c_cflag &= ~PARODD;
  } else {
    /* Odd */
    tios.c_cflag |= PARENB;
    tios.c_cflag |= PARODD;
  }

  /* Read the man page of termios if you need more information. */

  /* This field isn't used on POSIX systems
     tios.c_line = 0;
  */

  /* C_LFLAG      Line options

     ISIG Enable SIGINTR, SIGSUSP, SIGDSUSP, and SIGQUIT signals
     ICANON       Enable canonical input (else raw)
     XCASE        Map uppercase \lowercase (obsolete)
     ECHO Enable echoing of input characters
     ECHOE        Echo erase character as BS-SP-BS
     ECHOK        Echo NL after kill character
     ECHONL       Echo NL
     NOFLSH       Disable flushing of input buffers after
     interrupt or quit characters
     IEXTEN       Enable extended functions
     ECHOCTL      Echo control characters as ^char and delete as ~?
     ECHOPRT      Echo erased character as character erased
     ECHOKE       BS-SP-BS entire line on line kill
     FLUSHO       Output being flushed
     PENDIN       Retype pending input at next read or input char
     TOSTOP       Send SIGTTOU for background output

     Canonical input is line-oriented. Input characters are put
     into a buffer which can be edited interactively by the user
     until a CR (carriage return) or LF (line feed) character is
     received.

     Raw input is unprocessed. Input characters are passed
     through exactly as they are received, when they are
     received. Generally you'll deselect the ICANON, ECHO,
     ECHOE, and ISIG options when using raw input
  */

  /* Raw input */
  tios.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

  /* C_IFLAG      Input options

     Constant     Description
     INPCK        Enable parity check
     IGNPAR       Ignore parity errors
     PARMRK       Mark parity errors
     ISTRIP       Strip parity bits
     IXON Enable software flow control (outgoing)
     IXOFF        Enable software flow control (incoming)
     IXANY        Allow any character to start flow again
     IGNBRK       Ignore break condition
     BRKINT       Send a SIGINT when a break condition is detected
     INLCR        Map NL to CR
     IGNCR        Ignore CR
     ICRNL        Map CR to NL
     IUCLC        Map uppercase to lowercase
     IMAXBEL      Echo BEL on input line too long
  */
  if (ctx_rtu->parity == 'N') {
    /* None */
    tios.c_iflag &= ~INPCK;
  } else {
    tios.c_iflag |= INPCK;
  }

  /* Software flow control is disabled */
  tios.c_iflag &= ~(IXON | IXOFF | IXANY);

  /* C_OFLAG      Output options
     OPOST        Postprocess output (not set = raw output)
     ONLCR        Map NL to CR-NL

     ONCLR ant others needs OPOST to be enabled
  */

  /* Raw ouput */
  tios.c_oflag &= ~OPOST;

  /* C_CC         Control characters
     VMIN         Minimum number of characters to read
     VTIME        Time to wait for data (tenths of seconds)

     UNIX serial interface drivers provide the ability to
     specify character and packet timeouts. Two elements of the
     c_cc array are used for timeouts: VMIN and VTIME. Timeouts
     are ignored in canonical input mode or when the NDELAY
     option is set on the file via open or fcntl.

     VMIN specifies the minimum number of characters to read. If
     it is set to 0, then the VTIME value specifies the time to
     wait for every character read. Note that this does not mean
     that a read call for N bytes will wait for N characters to
     come in. Rather, the timeout will apply to the first
     character and the read call will return the number of
     characters immediately available (up to the number you
     request).

     If VMIN is non-zero, VTIME specifies the time to wait for
     the first character read. If a character is read within the
     time given, any read will block (wait) until all VMIN
     characters are read. That is, once the first character is
     read, the serial interface driver expects to receive an
     entire packet of characters (VMIN bytes total). If no
     character is read within the time allowed, then the call to
     read returns 0. This method allows you to tell the serial
     driver you need exactly N bytes and any read call will
     return 0 or N bytes. However, the timeout only applies to
     the first character read, so if for some reason the driver
     misses one character inside the N byte packet then the read
     call could block forever waiting for additional input
     characters.

     VTIME specifies the amount of time to wait for incoming
     characters in tenths of seconds. If VTIME is set to 0 (the
     default), reads will block (wait) indefinitely unless the
     NDELAY option is set on the port with open or fcntl.
  */
  /* Unused because we use open with the NDELAY option */
  tios.c_cc[VMIN] = 0;
  tios.c_cc[VTIME] = 0;

  if (tcsetattr(ctx->s, TCSANOW, &tios) < 0) {
    close(ctx->s);
    ctx->s = -1;
    return -1;
  }
#endif

  return 0;
}

int rtu_set_serial_mode(rtu_t& rtu, int mode) {
#if HAVE_DECL_TIOCSRS485
  modbus_rtu_t* ctx_rtu = ctx->backend_data;
  struct serial_rs485 rs485conf;

  if (mode == MODBUS_RTU_RS485) {
    // Get
    if (ioctl(ctx->s, TIOCGRS485, &rs485conf) < 0) {
      return -1;
    }
    // Set
    rs485conf.flags |= SER_RS485_ENABLED;
    if (ioctl(ctx->s, TIOCSRS485, &rs485conf) < 0) {
      return -1;
    }

    ctx_rtu->serial_mode = MODBUS_RTU_RS485;
    return 0;
  } else if (mode == MODBUS_RTU_RS232) {
    /* Turn off RS485 mode only if required */
    if (ctx_rtu->serial_mode == MODBUS_RTU_RS485) {
      /* The ioctl call is avoided because it can fail on some RS232 ports */
      if (ioctl(ctx->s, TIOCGRS485, &rs485conf) < 0) {
        return -1;
      }
      rs485conf.flags &= ~SER_RS485_ENABLED;
      if (ioctl(ctx->s, TIOCSRS485, &rs485conf) < 0) {
        return -1;
      }
    }
    ctx_rtu->serial_mode = MODBUS_RTU_RS232;
    return 0;
  }
#else
  fprintf(stderr, "This function isn't supported on your platform\n");
  return -1;
#endif
}



int rtu_get_serial_mode(rtu_t& rtu) {
#if HAVE_DECL_TIOCSRS485
  modbus_rtu_t* ctx_rtu = ctx->backend_data;
  return ctx_rtu->serial_mode;
#else
    fprintf(stderr, "This function isn't supported on your platform\n");
    return -1;
#endif
}

int rtu_get_rts(rtu_t& rtu) {
#if HAVE_DECL_TIOCM_RTS
  modbus_rtu_t* ctx_rtu = ctx->backend_data;
  return ctx_rtu->rts;
#else
  fprintf(stderr, "This function isn't supported on your platform\n");
  return -1;
#endif
}

int rtu_set_rts(rtu_t& rtu, int mode) {
#if HAVE_DECL_TIOCM_RTS
  modbus_rtu_t* ctx_rtu = ctx->backend_data;

  if (mode == MODBUS_RTU_RTS_NONE || mode == MODBUS_RTU_RTS_UP ||
    mode == MODBUS_RTU_RTS_DOWN) {
    ctx_rtu->rts = mode;

    /* Set the RTS bit in order to not reserve the RS485 bus */
    ctx_rtu->set_rts(ctx, ctx_rtu->rts != MODBUS_RTU_RTS_UP);

    return 0;
  } else {
    errno = EINVAL;
    return -1;
  }
#else
  fprintf(stderr, "This function isn't supported on your platform\n");
  return -1;
#endif
}

int rtu_set_custom_rts(rtu_t& rtu, void (*set_rts) (rtu_t& rtu, int on)) {
#if HAVE_DECL_TIOCM_RTS
  modbus_rtu_t* ctx_rtu = ctx->backend_data;
  ctx_rtu->set_rts = set_rts;
  return 0;
#else
    fprintf(stderr, "This function isn't supported on your platform\n");
    return -1;
#endif
}

int rtu_get_rts_delay(rtu_t& rtu) {
#if HAVE_DECL_TIOCM_RTS
  modbus_rtu_t* ctx_rtu;
  ctx_rtu = (modbus_rtu_t*)ctx->backend_data;
  return ctx_rtu->rts_delay;
#else
  fprintf(stderr, "This function isn't supported on your platform\n");
  return -1;
#endif
}

int rtu_set_rts_delay(rtu_t& rtu, int us) {
#if HAVE_DECL_TIOCM_RTS
  modbus_rtu_t* ctx_rtu;
  ctx_rtu = (modbus_rtu_t*)ctx->backend_data;
  ctx_rtu->rts_delay = us;
  return 0;
#else
  fprintf(stderr, "This function isn't supported on your platform\n");
  return -1;
#endif
}


void _rtu_close(rtu_t& rtu) {
  /* Restore line settings and close file descriptor in RTU mode */

#if defined(_WIN32)
  /* Revert settings */
  if (!SetCommState(rtu.w_ser.fd, &rtu.old_dcb)) {
    fprintf(stderr, "ERROR Couldn't revert to configuration (LastError %d)\n",
      (int)GetLastError());
  }

  if (!CloseHandle(rtu.w_ser.fd)) {
    fprintf(stderr, "ERROR Error while closing handle (LastError %d)\n",
      (int)GetLastError());
  }
#else
  if (ctx->s != -1) {
    tcsetattr(ctx->s, TCSANOW, &ctx_rtu->old_tios);
    close(ctx->s);
    ctx->s = -1;
  }
#endif
}

int _rtu_flush(rtu_t& rtu) {
#if defined(_WIN32)
  rtu.w_ser.n_bytes = 0;
  return (PurgeComm(rtu.w_ser.fd, PURGE_RXCLEAR) == FALSE);
#else
  return tcflush(rtu.s, TCIOFLUSH);
#endif
}

int _rtu_select(
  rtu_t& rtu,
  fd_set* rset,
  struct timeval* tv,
  int length_to_read) {
  int s_rc;
#if defined(_WIN32)
  s_rc = win32_ser_select(&(rtu.w_ser), length_to_read, tv);
  if (s_rc == 0) {
    return -1;
  }

  if (s_rc < 0) {
    return -1;
  }
#else
  while ((s_rc = select(ctx->s + 1, rset, NULL, NULL, tv)) == -1) {
    if (errno == EINTR) {
      if (ctx->debug) {
        fprintf(stderr, "A non blocked signal was caught\n");
      }
      /* Necessary after an error */
      FD_ZERO(rset);
      FD_SET(ctx->s, rset);
    } else {
      return -1;
    }
  }

  if (s_rc == 0) {
    /* Timeout */
    errno = ETIMEDOUT;
    return -1;
  }
#endif

  return s_rc;
}

bool new_rtu(rtu_t& rtu) {
  /* Check device argument */
  if (rtu.device == NULL || *(rtu.device) == 0) {
    fprintf(stderr, "The device string is empty\n");
    return false;
  }

  /* Check baud argument */
  if (rtu.baud == 0) {
    fprintf(stderr, "The baud rate value must not be zero\n");
    return false;
  }

  if (rtu.parity != 'N' && rtu.parity != 'E' && rtu.parity != 'O') {
    return false;
  }

#if HAVE_DECL_TIOCSRS485
  /* The RS232 mode has been set by default */
  ctx_rtu->serial_mode = MODBUS_RTU_RS232;
#endif

#if HAVE_DECL_TIOCM_RTS
  /* The RTS use has been set by default */
  ctx_rtu->rts = MODBUS_RTU_RTS_NONE;

  /* Calculate estimated time in micro second to send one byte */
  ctx_rtu->onebyte_time = 1000000 * (1 + data_bit + (parity == 'N' ? 0 : 1) + stop_bit) / baud;

  /* The internal function is used by default to set RTS */
  ctx_rtu->set_rts = _modbus_rtu_ioctl_rts;

  /* The delay before and after transmission when toggling the RTS pin */
  ctx_rtu->rts_delay = ctx_rtu->onebyte_time;
#endif

  rtu.confirmation_to_ignore = FALSE;

  return true;
}
