#include <csignal>

#include <atomic>
#include <iostream>
#include <algorithm>
#include <memory>
#include <thread>
#include <chrono>

#include <gos/arduino/test/tools/text.h>
#include <gos/arduino/test/tools/types.h>
#include <gos/arduino/test/tools/options.h>
#include <gos/arduino/test/tools/setting.h>
#include <gos/arduino/test/tools/exception.h>
#include <gos/arduino/test/tools/modbus/gam.h>
#include <gos/arduino/test/tools/modbus/arduinoslave.h>

#include <gos/arduino/test/tools/version.h>

#define GOS_ARDUINO_TEST_TOOLS_MODBUS_SLAVE_CLI "slave"

#define GOS_ARDUINO_TEST_TOOLS_MODBUS_SLAVE_ID 1

#define GOS_ARDUINO_TEST_TOOLS_MODBUS_BUFFER_SIZE 256

namespace po = ::boost::program_options;

namespace gatt = ::gos::arduino::test::tools;
namespace gatto = ::gos::arduino::test::tools::options;
namespace gatts = ::gos::arduino::test::tools::setting;
namespace gattt = ::gos::arduino::test::tools::text;

namespace slave {
typedef std::unique_ptr<gatt::modbus::slave> Pointer;
enum class type { arduino, gos };
}

const uint16_t BufferSize = GOS_ARDUINO_TEST_TOOLS_MODBUS_BUFFER_SIZE;

static uint16_t _slave_id = GOS_ARDUINO_TEST_TOOLS_MODBUS_SLAVE_ID;

namespace slave {
static type _type = type::gos;
}

void create(po::options_description& description);
bool handle(po::variables_map& varmap);

std::atomic_bool _go;

static void sigint(int s);

int main(int argc, char* argv[]) {

  try {
    po::options_description optdescript(gatto::general::Name);
    gatto::general::create(optdescript);

    po::options_description communicationdescript(gatto::communication::Name);
    gatto::communication::create(communicationdescript);

    po::options_description timer(gatto::timer::Name);
    gatto::timer::create(timer);

    po::options_description slave(GOS_ARDUINO_TEST_TOOLS_MODBUS_SLAVE_CLI);
    create(slave);

    po::options_description clioptions(
      GOS_ARDUINO_TEST_TOOLS_MODBUS_SLAVE_CLI " " GOST_USAGE,
      GOS_PO_LINE_LENGTH);
    clioptions
      .add(optdescript)
      .add(communicationdescript)
      .add(timer)
      .add(slave)
      ;

    po::variables_map varmap;

    /* First round with CLI parser only */
    po::command_line_parser cmdlinparser(argc, argv);
    po::parsed_options parseopt = cmdlinparser.options(clioptions)
      .style(po::command_line_style::default_style).run();
    po::store(parseopt, varmap);

    if (gatto::handling::help(clioptions, varmap)) {
      return GOS_PO_EXIT_HELP;
    }

    std::string version = gatt::version::generate(
      gatt::version::WithBuildDateTime);
    std::string name = GOS_ARDUINO_TEST_TOOLS_MODBUS_SLAVE_CLI;

    if (gatto::handling::version(varmap, name, version)) {
      return GOS_PO_EXIT_VERSION;
    }

    gatto::handling::verbosity(varmap);

    /* second round */
    po::notify(varmap);

    /* Create the configuration file options description */
    po::options_description configfdesc;
    configfdesc
      .add(communicationdescript)
      .add(timer)
      .add(slave)
      ;
    if (gatto::handling::file(configfdesc, varmap) == EXIT_FAILURE) {
      return EXIT_FAILURE;
    }

    gatto::handling::communication(varmap);
    handle(varmap);
  }
  catch (gatt::exception & er) {
    std::cerr << "Drill Link tool exception: "
      << er.what() << std::endl;
    return EXIT_FAILURE;
  }
  catch (po::error & er) {
    std::cerr << "Boost parsing program option exception: "
      << er.what() << std::endl;
    return EXIT_FAILURE;
  }
  catch (std::exception & ex) {
    std::cerr << "General parsing option exception: "
      << ex.what() << std::endl;
    return EXIT_FAILURE;
  }
  catch (...) {
    std::cerr << "General parsing option exception" << std::endl;
    return EXIT_FAILURE;
  }

  gatt::initialize();

  _Serial serial(
    gatts::communication::serial::port,
    gatts::communication::serial::baud,
    gatts::communication::serial::data::bits,
    gatts::communication::serial::stop::bits,
    gatts::communication::serial::parity);

  if (serial.initialize()) {
    std::cout << "Serial successfully initialize" << std::endl;
  } else {
    std::cerr << "Serial failed to initialize" << std::endl;
    return EXIT_FAILURE;
  }

  slave::Pointer slave;

  switch (slave::_type) {
  case slave::type::arduino:
    slave = std::make_unique<gatt::modbus::modbusslave>(serial, _slave_id);
    if (slave) {
      std::cout << "An Arduino Modbus slave instance has been created"
        << std::endl;
    } else {
      std::cerr << "Failed to create Arduino Modbus slave instance"
        << std::endl;
      return EXIT_FAILURE;
    }
    break;
  case slave::type::gos:
    slave = std::make_unique<gatt::modbus::gam>(serial, _slave_id, BufferSize);
    if (slave) {
      std::cout << "GOS Modbus slave instance has been created" << std::endl;
    } else {
      std::cerr << "Failed to create GOS ATL Modbus slave instance"
        << std::endl;
      return EXIT_FAILURE;
    }
    break;
  }

  slave->begin(gatt::setting::communication::serial::baud);

  signal(SIGINT, sigint);

  _go.store(true);

  if (gatts::timing::interval::milliseconds::loop) {
    std::cout << "Starting the loop with delay of "
      << gatts::timing::interval::milliseconds::loop << " of milliseconds"
      << std::endl;
  } else {
    std::cout << "Starting a continuous loop without any delay" << std::endl;
  }
  std::chrono::milliseconds delay(gatts::timing::interval::milliseconds::loop);
  while (_go.load()) {
    serial.loop();
    slave->loop();
    if (gatts::timing::interval::milliseconds::loop > 0) {
      std::this_thread::sleep_for(delay);
    }
  }

  std::cout << "Shutdown the serial." << std::endl;
  serial.shutdown();

  std::cout << "It is finished!" << std::endl;
}

void create(po::options_description& description) {
  description.add_options()
    (gattt::option::composite::SlaveId,
      po::value(&(_slave_id))
      ->default_value(GOS_ARDUINO_TEST_TOOLS_MODBUS_SLAVE_ID),
      gattt::option::description::SlaveId)
    (GOST_SLAVE GOST_DASH GOST_TYPE GOST_OSC GOST_OS_T,
      po::value<std::string>()->default_value("GOS"),
      "Slave type (GOS or Arduino)");
}

bool handle(po::variables_map& varmap) {
  if (varmap.count(GOST_SLAVE GOST_DASH GOST_TYPE)) {
    std::string text = varmap[GOST_SLAVE GOST_DASH GOST_TYPE].as<std::string>();
    std::transform(text.begin(), text.end(), text.begin(), ::tolower);
    if (text.compare("gos") == 0) {
      std::cout << "Setting modbus slave type to GOS" << std::endl;
      slave::_type = slave::type::gos;
    } else if(text.compare("arduino") == 0) {
      std::cout << "Setting modbus slave type to Arduino" << std::endl;
      slave::_type = slave::type::arduino;
    } else {
      std::cerr << "Unsupported slave type '" << text << "'" << std::endl;
      return false;
    }
  }
  return true;
}


static void sigint(int s) {
  std::cout << "Got signal, terminating the go!" << std::endl;
  _go.store(false);
}
