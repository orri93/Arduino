#ifndef GOS_ARDUINO_TEST_TOOL_TEXT_H_
#define GOS_ARDUINO_TEST_TOOL_TEXT_H_

#define GOST_NUL ""

#define GOST_ADDRESS "address"
#define GOST_BAUD "baud"
#define GOST_CONFIG "config"
#define GOST_CONFIGURATION "configuration"
#define GOST_CONNECTION "connection"
#define GOST_DEBUG "debug"
#define GOST_FILE "file"
#define GOST_FLAG "flag"
#define GOST_HELP "help"
#define GOST_ID "id"
#define GOST_INTERVAL "interval"
#define GOST_IP "ip"
#define GOST_MAXIMUM "maximum"
#define GOST_LISTEN "listen"
#define GOST_LOOP "loop"
#define GOST_NOISE "noise"
#define GOST_PORT "port"
#define GOST_QUIET "quiet"
#define GOST_RANDOM "random"
#define GOST_SERIAL "serial"
#define GOST_SILENT "silent"
#define GOST_SLAVE "slave"
#define GOST_TABLE "table"
#define GOST_TOOL "tool"
#define GOST_TYPE "type"
#define GOST_USAGE "usage"
#define GOST_VALUE "value"
#define GOST_VERBOSE "verbose"
#define GOST_VERSION "version"
#define GOST_WAIT "wait"
#define GOST_WORK "work"

#define GOST_C_COMMUNICATION "Communication"
#define GOST_C_GENERAL "General"
#define GOST_C_TIMER "Timer"
#define GOST_C_TOOL "Tool"

#define GOST_OSC ","
#define GOST_DASH "-"

#define GOST_OS_1 "1"
#define GOST_OS_2 "2"
#define GOST_OS_3 "3"
#define GOST_OS_4 "4"

//#define GOST_OS_A "a"
#define GOST_OS_B "b"
#define GOST_OS_C "c"
#define GOST_OS_D "d"
#define GOST_OS_F "f"
#define GOST_OS_H "h"
#define GOST_OS_I "i"
//#define GOST_OS_N "n"
//#define GOST_OS_M "m"
#define GOST_OS_P "p"
//#define GOST_OS_R "r"
#define GOST_OS_T "t"
#define GOST_OS_V "v"

namespace gos {
namespace arduino {
namespace test {
namespace tools {
namespace text {

extern const char* Address;
extern const char* Baud;
extern const char* Config;
extern const char* Configuration;
extern const char* Connection;
extern const char* Debug;
extern const char* Flag;
extern const char* File;
extern const char* Help;
extern const char* Id;
extern const char* Interval;
extern const char* Ip;
extern const char* Listen;
extern const char* Loop;
extern const char* Noise;
extern const char* Maximum;
extern const char* Quiet;
extern const char* Port;
extern const char* Random;
extern const char* Serial;
extern const char* Silent;
extern const char* Slave;
extern const char* Table;
extern const char* Tool;
extern const char* Type;
extern const char* Usage;
extern const char* Value;
extern const char* Verbose;
extern const char* Version;
extern const char* Wait;
extern const char* Work;

namespace capitalised {
namespace first {
extern const char* Communication;
extern const char* General;
extern const char* Timer;
extern const char* Tool;
}
}

namespace option {
extern const char* ConfigurationFile;
extern const char* LoopInterval;
extern const char* SlaveId;

namespace composite {
extern const char* Config;
extern const char* Debug;
extern const char* File;
extern const char* Help;
//extern const char* Noise;
extern const char* Port;
extern const char* Random;
extern const char* SerialBaud;
extern const char* SerialPort;
extern const char* SlaveId;
//extern const char* Table;
extern const char* Version;
}
namespace description {
extern const char* Config;
extern const char* ConfigurationFile;
extern const char* Debug;
extern const char* FlagValue;
extern const char* Help;
extern const char* SerialPort;
extern const char* Noise;
extern const char* SerialBaud;
extern const char* Port;
extern const char* Quiet;
extern const char* Random;
extern const char* Silent;
extern const char* SlaveId;
extern const char* Table;
extern const char* Version;
extern const char* Verbose;
extern const char* LoopInterval;
}
}

} // namespace text
} // namespace tools
} // namespace test
} // namespace arduino 
} // namespace gos

#endif
