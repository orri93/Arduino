#ifndef GOS_ARDUINO_TOOL_OPTIONS_H_
#define GOS_ARDUINO_TOOL_OPTIONS_H_

#include <cstdlib>

#include <boost/program_options.hpp>

#define GOS_PO_LINE_LENGTH 80
#define GOS_PO_EXIT_HELP EXIT_FAILURE
#define GOS_PO_EXIT_VERSION EXIT_FAILURE

namespace gos {
namespace arduino {
namespace tools {
namespace options {

namespace general {
extern const char* Name;
void create(
  ::boost::program_options::options_description& description,
  const bool& file = false);
}

namespace communication {
extern const char* Name;
void create(::boost::program_options::options_description& description);
}

namespace timer {
extern const char* Name;
void create(::boost::program_options::options_description& description,
  const int& defaultloopinterval);
}

namespace tool {
extern const char* Name;
void create(::boost::program_options::options_description& description);
}


namespace handling {
bool help(
  ::boost::program_options::options_description& description,
  ::boost::program_options::variables_map& map);
bool version(
  ::boost::program_options::variables_map& map,
  const std::string& name,
  const std::string& version);
void verbosity(::boost::program_options::variables_map& map);
int file(
  ::boost::program_options::options_description& filedescription,
  ::boost::program_options::variables_map& map);
void reporterror(const ::std::string& errortext);
::std::string lasterrortext();
void communication(::boost::program_options::variables_map& map);
void tool(::boost::program_options::variables_map& map);
}

}
}
}
}

#endif
