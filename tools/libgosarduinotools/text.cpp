#include <gos/arduino/tools/text.h>

namespace gos {
namespace arduino {
namespace tools {
namespace text {

const char* Address = GOST_ADDRESS;
const char* Config = GOST_CONFIG;
const char* Configuration = GOST_CONFIGURATION;
const char* Connection = GOST_CONNECTION;
const char* Debug = GOST_DEBUG;
const char* Flag = GOST_FILE;
const char* Help = GOST_HELP;
const char* Id = GOST_ID;
const char* Interval = GOST_INTERVAL;
const char* Ip = GOST_IP;
const char* Listen = GOST_LISTEN;
const char* Loop = GOST_LOOP;
const char* Noise = GOST_NOISE;
const char* Maximum = GOST_MAXIMUM;
const char* Quiet = GOST_QUIET;
const char* File = GOST_FILE;
const char* Port = GOST_PORT;
const char* Random = GOST_RANDOM;
const char* Silent = GOST_SILENT;
const char* Slave = GOST_SLAVE;
const char* Table = GOST_TABLE;
const char* Tool = GOST_TOOL;
const char* Usage = GOST_USAGE;
const char* Value = GOST_VALUE;
const char* Verbose = GOST_VERBOSE;
const char* Version = GOST_VERSION;
const char* Wait = GOST_WAIT;
const char* Work = GOST_WORK;

namespace capitalised {
namespace first {
const char* Communication = GOST_C_COMMUNICATION;
const char* General = GOST_C_GENERAL;
const char* Timer = GOST_C_TIMER;
const char* Tool = GOST_C_TOOL;
}
}

namespace option {
const char* ConfigurationFile = GOST_CONFIGURATION GOST_DASH GOST_FILE;
const char* LoopInterval = GOST_LOOP GOST_DASH GOST_INTERVAL;
const char* SlaveId = GOST_SLAVE GOST_DASH GOST_ID;
namespace composite {
const char* Config = GOST_CONFIG GOST_OSC GOST_OS_C;
const char* Debug = GOST_DEBUG GOST_DASH GOST_OS_D;
const char* File = GOST_FILE GOST_OSC GOST_OS_F;
const char* Help = GOST_HELP GOST_OSC GOST_OS_H;
//const char* Noise = GOST_NOISE GOST_OSC GOST_OS_N;
//const char* Port = GOST_PORT GOST_OSC GOST_OS_P;
//const char* Random = GOST_RANDOM GOST_OSC GOST_OS_R;
const char* SerialBaud = GOST_SERIAL GOST_DASH GOST_BAUD GOST_OSC GOST_OS_B;
const char* SerialPort = GOST_SERIAL GOST_DASH GOST_PORT GOST_OSC GOST_OS_P;
const char* SlaveId = GOST_SLAVE GOST_DASH GOST_ID GOST_OSC GOST_OS_I;
//const char* Table = GOST_TABLE GOST_OSC GOST_OS_T;
const char* Version = GOST_VERSION GOST_OSC GOST_OS_V;
}
namespace description {
const char* Config = "configuration file";
const char* ConfigurationFile = Config;
const char* Debug = "Provide additional debug information";
const char* FlagValue = "value to return as flag";
const char* Help = "produce help message";
const char* Noise = "noise for tool values";
const char* Port = "modbus listen TCP port";
const char* Quiet = Silent;
//const char* Random = "random values for the tools";
const char* SerialBaud = "Modbus serial baud";
const char* SerialPort = "Modbus serial port";
const char* Silent = "inhibit all outputs besides the requested";
const char* SlaveId = "modbus slave ID";
const char* Table = "output as table";
const char* Version = "print version string";
const char* Verbose = "print more information about the progress";
const char* LoopInterval = "loop interval";
}
}

}
}
}
}
