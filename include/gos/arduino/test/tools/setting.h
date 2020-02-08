#ifndef GOS_ARDUINO_TEST_TOOL_SETTINGS_H_
#define GOS_ARDUINO_TEST_TOOL_SETTINGS_H_

#include <gos/arduino/test/tools/types.h>

namespace gos {
namespace arduino {
namespace test {
namespace tools {
namespace setting {

extern ::gos::arduino::test::tools::types::level verbosity;

namespace communication {
namespace serial {
extern std::string port;
extern int baud;
namespace data {
extern int bits;
} // namespace data
namespace stop {
extern int bits;
} // namespace stop
extern char parity;
} // namespace serial
} // namespace communication

namespace timing {
namespace interval {
namespace milliseconds {
extern int loop;
} // namespace milliseconds
} // namespace interval
} // namespace timing

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
} // namespace file
} // namespace configuration

} // namespace setting
} // namespace tools
} // namespace test
} // namespace arduino 
} // namespace gos

#endif
