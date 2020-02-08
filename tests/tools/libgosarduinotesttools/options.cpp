#include <cstdlib>

#include <string>

#include <iostream>

#include <boost/filesystem.hpp>

#include <gos/arduino/test/tools/default.h>
#include <gos/arduino/test/tools/setting.h>
#include <gos/arduino/test/tools/options.h>
#include <gos/arduino/test/tools/text.h>

namespace po = ::boost::program_options;

namespace gatt = ::gos::arduino::test::tools;
namespace gattt = ::gos::arduino::test::tools::text;

namespace gattd = ::gos::arduino::test::tools::default;
namespace gatttcf = ::gos::arduino::test::tools::text::capitalised::first;
namespace gattto = ::gos::arduino::test::tools::text::option;
namespace gatts = ::gos::arduino::test::tools::setting;
namespace gattscf = ::gos::arduino::test::tools::setting::configuration::file;

namespace gos {
namespace arduino {
namespace test {
namespace tools {
namespace options {

namespace general {
const char* Name = gatttcf::General;
void create(po::options_description& description, const bool& file) {
  description.add_options()
    (gattto::composite::Help, gattto::description::Help)
    (gattto::composite::Version, gattto::description::Version)
    (gattt::Verbose, gattto::description::Verbose)
    (gattt::Silent, gattto::description::Silent)
    (gattt::Quiet, gattto::description::Quiet);
  if (file) {
    description.add_options()
      (gattto::composite::Config, po::value<std::string>(),
        gattto::description::Config);
  }
}
} // namespace general

namespace communication {
const char* Name = gatttcf::Communication;
void create(po::options_description& description) {
  description.add_options()
    (gattt::option::composite::SerialPort,
      po::value(&(gatts::communication::serial::port))
      ->default_value(gatt::default::communication::serial::Port),
      gattt::option::description::SerialPort)
    (gattt::option::composite::SerialBaud,
      po::value(&(gatts::communication::serial::baud))
      ->default_value(gatt::default::communication::serial::Baud),
      gattt::option::description::SerialBaud);
}
} // namespace communication

namespace timer {
const char* Name = gatttcf::Timer;
void create(::boost::program_options::options_description& description,
  const int& defaultloopinterval) {
  description.add_options()
    (gattt::option::LoopInterval,
      po::value(&(gatts::timing::interval::milliseconds::loop))
      ->default_value(defaultloopinterval),
      gattt::option::description::LoopInterval);
}
void create(::boost::program_options::options_description& description) {
  create(description, gatt::default::timing::interval::milliseconds::Loop);
}
} // namespace timer

#ifdef GOS_NOT_USED_YET
namespace tool {
const char* Name = gatttcf::Tool;
void create(::boost::program_options::options_description& description) {
  description.add_options()
    (gattt::option::composite::Table, gattt::option::description::Table);
}
} // namespace tool
#endif

namespace handling {

static std::string lasterrortext_;

bool help(
  ::boost::program_options::options_description& description,
  ::boost::program_options::variables_map& map) {
  if (map.count(gattt::Help)) {
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
  if (map.count(gattt::Version)) {
    std::cout << name << " version " << version << std::endl;
    return true;
  } else {
    return false;
  }
}

void verbosity(::boost::program_options::variables_map& map) {
  if (map.count(gattt::Quiet) || map.count(gattt::Silent)) {
    gatts::verbosity = gatt::types::level::silent;
  } else if (map.count(gattt::Verbose)) {
    gatts::verbosity = gatt::types::level::verbose;
  }
}

int file(
  ::boost::program_options::options_description& filedescription,
  ::boost::program_options::variables_map& map) {
  if (map.count(gattt::Config)) {
    if (!gattscf::apply(map[gattt::Config].as<std::string>())) {
      lasterrortext_ = gattscf::lasterrortext();
      std::cerr << lasterrortext_ << std::endl;
      return EXIT_FAILURE;
    }
  } else if (map.count(gattto::ConfigurationFile)) {
    if (!gattscf::apply(map[gattto::ConfigurationFile].
      as<std::string>())) {
      lasterrortext_ = gattscf::lasterrortext();
      std::cerr << lasterrortext_ << std::endl;
      return EXIT_FAILURE;
    }
  }
  if (gattscf::iswithfile()) {
    if (gatts::isverbose()) {
      std::cout << "Using '" << gattscf::configurationfilename()
        << "' as configuration file" << std::endl;
    }
    std::ifstream& stream = gattscf::stream();
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
  if (map.count(gattt::Port)) {
    gatts::endpoint.second = map[gattt::Port].as<uint16_t>();
  }
  if (map.count(gattt::option::ListenIpAddress)) {
    gatts::endpoint.first = map[gattt::option::ListenIpAddress].as<std::string>();
  }
  if (map.count(gattt::option::MaximumConnection)) {
    gatts::maximum_number_of_connection =
      map[gattt::option::MaximumConnection].as<int>();
  }
  if (map.count(gattt::option::SlaveId)) {
    gatts::slave_id = map[gattt::option::SlaveId].as<int>();
  }
#endif
}

#ifdef GOS_NOT_USED_YET
void tool(po::variables_map& map) {
  if (map.count(gattt::Table)) {
  }
}
#endif

} // namespace handling

} // namespace options
} // namespace tools
} // namespace test
} // namespace arduino 
} // namespace gos

