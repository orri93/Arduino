#include <sstream>
#include <iostream>
#include <memory>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#ifdef GOS_NOT_YET_NEEDED
#include <modbus.h>
#endif

#include <gos/arduino/test/tools/setting.h>
#include <gos/arduino/test/tools/exception.h>
#include <gos/arduino/test/tools/default.h>
#include <gos/arduino/test/tools/types.h>
#include <gos/arduino/test/tools/text.h>

#ifdef NOV_USE_BOOST_MAKE_UNIQUE
namespace mu = ::boost;
#else
namespace mu = ::std;
#endif

namespace gatt = ::gos::arduino::test::tools;
namespace gattt = ::gos::arduino::test::tools::text;

namespace gos {
namespace arduino {
namespace test {
namespace tools {
namespace setting {

gatt::types::level verbosity = gatt::default::Verbosity;

namespace communication {
namespace serial {
std::string port = gatt::default::communication::serial::Port;
int baud = gatt::default::communication::serial::Baud;
namespace data {
int bits = gatt::default::communication::serial::data::Bits;
} // namespace data
namespace stop {
int bits = gatt::default::communication::serial::stop::Bits;
} // namespace stop
char parity = gatt::default::communication::serial::Parity;
} // namespace serial
} // namespace communication


namespace timing {
namespace interval {
namespace milliseconds {
int loop = gatt::default::timing::interval::milliseconds::Loop;
} // namespace milliseconds
} // namespace interval
} // namespace timing


void create() {
}

bool issilent() { return verbosity == gatt::types::level::silent; }
bool isnormal() { return verbosity != gatt::types::level::silent; }
bool isverbose() { return verbosity == gatt::types::level::verbose; }

namespace configuration {
namespace file {
namespace details {
typedef std::unique_ptr<std::ifstream> FileStreamPointer;
static std::string last_error_;
static std::string configuration_file_name_;
static FileStreamPointer configuration_stream_;
static bool is_configurateion_file_valid_;
bool check(std::string& filename, const char* name) {
  ::boost::algorithm::trim(filename);
  std::stringstream errstrm;
  if (filename.size() > 0) {
    if (::boost::filesystem::exists(filename)) {
      return true;
    } else {
      errstrm << "The specified " << name << " file '" << filename
        << "' doesn't exist" << std::ends;
    }
  } else {
    errstrm << "The specified " << name
      << " file name is not valid" << std::ends;
  }
  details::last_error_ = errstrm.str();
  return false;
}
}

bool apply(const std::string& configurationfilename) {
  details::configuration_file_name_ = configurationfilename;
  std::stringstream errstrm;
  if (details::check(details::configuration_file_name_, gattt::Configuration)) {
    details::configuration_stream_ =
      mu::make_unique<std::ifstream>(details::configuration_file_name_);
    if (details::configuration_stream_) {
      if (details::configuration_stream_->is_open()) {
        details::is_configurateion_file_valid_ = true;
        return true;
      } else {
        errstrm << "The specified configuration file '"
          << details::configuration_file_name_
          << "' is not opened" << std::ends;
      }
    } else {
      errstrm << "Failed to open the specified configuration file '"
        << details::configuration_file_name_ << "'" << std::ends;
    }
    details::last_error_ = errstrm.str();
  }
  return false;
}

std::string configurationfilename() {
  return details::configuration_file_name_;
}

std::string& lasterrortext() {
  return details::last_error_;
}

std::ifstream& stream() {
  if (details::configuration_stream_) {
    return *(details::configuration_stream_);
  } else {
    throw ::gos::arduino::test::tools::exception(
      "Configuration file stream is undefined");
  }
}

bool iswithfile() {
  return details::is_configurateion_file_valid_;
}

} // namespace file
} // namespace configuration


} // namespace setting
} // namespace tools
} // namespace test
} // namespace arduino 
} // namespace gos
