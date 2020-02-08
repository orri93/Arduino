#include <cstdlib>

#include <string>

#include <iostream>

#include <boost/filesystem.hpp>

#include <gos/arduino/tools/default.h>
#include <gos/arduino/tools/setting.h>
#include <gos/arduino/tools/options.h>
#include <gos/arduino/tools/text.h>

namespace po = ::boost::program_options;

namespace gat = ::gos::arduino::tools;
namespace gatt = ::gos::arduino::tools::text;

namespace gatd = ::gos::arduino::tools::default;
namespace gattcf = ::gos::arduino::tools::text::capitalised::first;
namespace gatto = ::gos::arduino::tools::text::option;
namespace gats = ::gos::arduino::tools::setting;
namespace gatscf = ::gos::arduino::tools::setting::configuration::file;

namespace gos {
namespace arduino {
namespace tools {
namespace options {

namespace general {
const char* Name = gattcf::General;
void create(po::options_description& description, const bool& file) {
  description.add_options()
    (gatto::composite::Help, gatto::description::Help)
    (gatto::composite::Version, gatto::description::Version)
    (gatt::Verbose, gatto::description::Verbose)
    (gatt::Silent, gatto::description::Silent)
    (gatt::Quiet, gatto::description::Quiet);
  if (file) {
    description.add_options()
      (gatto::composite::Config, po::value<std::string>(),
        gatto::description::Config);
  }
}
}

namespace communication {
const char* Name = gattcf::Communication;
void create(po::options_description& description) {
  description.add_options()
    (gatt::option::composite::SerialPort,
      po::value(&(gats::communication::serial::port))
      ->default_value(gat::default::communication::serial::Port),
      gatt::option::description::SerialPort)
    (gatt::option::composite::SerialBaud,
      po::value(&(gats::communication::serial::baud))
      ->default_value(gat::default::communication::serial::Baud),
      gatt::option::description::SerialBaud)
    (gatt::option::composite::SlaveId,
      po::value(&(gats::slave::id)) ->default_value(gat::default::slave::Id),
      gatt::option::description::SlaveId);
}
}

namespace timer {
const char* Name = gattcf::Timer;
void create(::boost::program_options::options_description& description,
  const int& defaultloopinterval) {
  description.add_options()
    (gatt::option::LoopInterval,
      po::value(&(gats::timing::interval::milliseconds::loop))
      ->default_value(defaultloopinterval),
      gatt::option::description::LoopInterval);
}
}

#ifdef GOS_NOT_USED_YET
namespace tool {
const char* Name = gattcf::Tool;
void create(::boost::program_options::options_description& description) {
  description.add_options()
    (gatt::option::composite::Table, gatt::option::description::Table);
}
}
#endif


namespace handling {

static std::string lasterrortext_;

bool help(
  ::boost::program_options::options_description& description,
  ::boost::program_options::variables_map& map) {
  if (map.count(gatt::Help)) {
    std::cout << description << std::endl;
    return true;
  } else {
    return false;
  }
}

bool version(
  ::boost::program_options::variables_map& map,
  const std::string& name,
  const std::string& version) {
  if (map.count(gatt::Version)) {
    std::cout << name << " version " << version << std::endl;
    return true;
  } else {
    return false;
  }
}

void verbosity(::boost::program_options::variables_map& map) {
  if (map.count(gatt::Quiet) || map.count(gatt::Silent)) {
    gats::verbosity = gat::types::level::silent;
  } else if (map.count(gatt::Verbose)) {
    gats::verbosity = gat::types::level::verbose;
  }
}

int file(
  ::boost::program_options::options_description& filedescription,
  ::boost::program_options::variables_map& map) {
  if (map.count(gatt::Config)) {
    if (!gatscf::apply(map[gatt::Config].as<std::string>())) {
      lasterrortext_ = gatscf::lasterrortext();
      std::cerr << lasterrortext_ << std::endl;
      return EXIT_FAILURE;
    }
  } else if (map.count(gatto::ConfigurationFile)) {
    if (!gatscf::apply(map[gatto::ConfigurationFile].
      as<std::string>())) {
      lasterrortext_ = gatscf::lasterrortext();
      std::cerr << lasterrortext_ << std::endl;
      return EXIT_FAILURE;
    }
  }
  if (gatscf::iswithfile()) {
    if (gats::isverbose()) {
      std::cout << "Using '" << gatscf::configurationfilename()
        << "' as configuration file" << std::endl;
    }
    std::ifstream& stream = gatscf::stream();
    po::store(po::parse_config_file(stream, filedescription), map);
    po::notify(map);
  }
  return EXIT_SUCCESS;
}

void reporterror(const ::std::string& errortext) {
  lasterrortext_ = errortext;
}

::std::string lasterrortext() {
  return lasterrortext_;
}

void communication(po::variables_map& map) {
#ifdef NOT_NEEDED
  if (map.count(gatt::Port)) {
    gats::endpoint.second = map[gatt::Port].as<uint16_t>();
  }
  if (map.count(gatt::option::ListenIpAddress)) {
    gats::endpoint.first = map[gatt::option::ListenIpAddress].as<std::string>();
  }
  if (map.count(gatt::option::MaximumConnection)) {
    gats::maximum_number_of_connection =
      map[gatt::option::MaximumConnection].as<int>();
  }
  if (map.count(gatt::option::SlaveId)) {
    gats::slave_id = map[gatt::option::SlaveId].as<int>();
  }
#endif
}

void tool(po::variables_map& map) {
  if (map.count(gatt::Table)) {
  }
}

}

}
}
}
}

