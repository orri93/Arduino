#ifndef GOS_ARDUINO_TEST_TOOL_OPTIONS_H_
#define GOS_ARDUINO_TEST_TOOL_OPTIONS_H_

#include <cstdlib>

#include <boost/program_options.hpp>

#ifndef GOS_PO_LINE_LENGTH
#define GOS_PO_LINE_LENGTH 80
#endif
#ifndef GOS_PO_EXIT_HELP
#define GOS_PO_EXIT_HELP EXIT_FAILURE
#endif
#ifndef GOS_PO_EXIT_VERSION
#define GOS_PO_EXIT_VERSION EXIT_FAILURE
#endif

namespace gos {
namespace arduino {
namespace test {
namespace tools {
namespace options {

namespace general {
extern const char* Name;
void create(
  ::boost::program_options::options_description& description,
  const bool& file = false);
} // namespace general

namespace communication {
extern const char* Name;
void create(::boost::program_options::options_description& description);
} // namespace communication

namespace timer {
extern const char* Name;
void create(::boost::program_options::options_description& description,
  const int& defaultloopinterval);
void create(::boost::program_options::options_description& description);
} // namespace timer

#ifdef GOS_NOT_USED_YET
namespace tool {
extern const char* Name;
void create(::boost::program_options::options_description& description);
} // namespace tool
#endif

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
#ifdef GOS_NOT_USED_YET
void tool(::boost::program_options::variables_map& map);
#endif
} // namespace handling

} // namespace options
} // namespace tools
} // namespace test
} // namespace arduino 
} // namespace gos

#endif
