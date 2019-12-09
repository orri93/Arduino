#include <sstream>
#include <iostream>
#include <memory>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#ifdef GOS_NOT_YET_NEEDED
#include <modbus.h>
#endif

#include <gos/arduino/tools/setting.h>
#include <gos/arduino/tools/exception.h>
#include <gos/arduino/tools/default.h>
#include <gos/arduino/tools/types.h>
#include <gos/arduino/tools/text.h>

#ifdef NOV_USE_BOOST_MAKE_UNIQUE
namespace mu = ::boost;
#else
namespace mu = ::std;
#endif

namespace gat = ::gos::arduino::tools;
namespace gatt = ::gos::arduino::tools::text;

namespace gos {
namespace arduino {
namespace tools {
namespace setting {

level verbosity = level::normal;

namespace communication {
extern std::string serialport = gat::default::serial::string::Port;
extern int serialbaud = gat::default::serial::Baud;
int slave_id = gat::default::slave::Id;
}

namespace timing {
namespace interval {
namespace milliseconds {
extern int loop = 0;
}
}
}

void create() {
}

bool issilent() { return verbosity == level::silent; }
bool isnormal() { return verbosity != level::silent; }
bool isverbose() { return verbosity == level::verbose; }

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
  if (details::check(details::configuration_file_name_, gatt::Configuration)) {
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
    throw ::gos::arduino::tools::exception(
      "Configuration file stream is undefined");
  }
}

bool iswithfile() {
  return details::is_configurateion_file_valid_;
}

}
}



}
}
}
}