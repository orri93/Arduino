#ifndef GOS_ARDUINO_TOOL_SETTINGS_H_
#define GOS_ARDUINO_TOOL_SETTINGS_H_

#include <boost/program_options.hpp>

#include <gos/arduino/tools/types.h>>

namespace gos {
namespace arduino {
namespace tools {
namespace setting {

enum class level {
  silent,
  normal,
  verbose
};

extern level verbosity;

namespace communication {
extern std::string serialport;
extern int serialbaud;
extern int slave_id;
}

namespace timing {
namespace interval {
namespace milliseconds {
extern int loop;
}
}
}

void create();

bool issilent();
bool isnormal();
bool isverbose();

namespace configuration {
namespace file {
bool check(std::string& filename, const char* name);
bool apply(const std::string& configurationfilename);
std::string configurationfilename();
std::string& lasterrortext();
std::ifstream& stream();
bool iswithfile();
}
}

}
}
}
}

#endif
