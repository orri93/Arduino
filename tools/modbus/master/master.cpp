#include <cassert>
#include <cstring>

#include <iostream>
//#include <random>
#include <string>
#include <atomic>
#include <regex>

#ifdef WIN32
#include <WinSock2.h>
#include <Windows.h>
#endif

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>

#include <modbus.h>

#include <gos/arduino/tools/text.h>
#include <gos/arduino/tools/options.h>
#include <gos/arduino/tools/setting.h>
#include <gos/arduino/tools/exception.h>

#define GOS_ARDUINO_TOOLS_MASTER_NAME "master"
#define GOS_ARDUINO_TOOLS_MASTER_LOG_FILE GOS_ARDUINO_TOOLS_MASTER_NAME ".log"
#define GOS_ARDUINO_TOOLS_MASTER_INTERVAL 2000

#define GOS_ARDT_SET_FL modbus_set_float
#define GOS_ARDT_GET_FL modbus_get_float

#define GOS_ARDT_COILS 1
//#define GOS_ARDT_DISCRETE 0
#define GOS_ARDT_HOLDING 13
#define GOS_ARDT_INPUT 3

#define GOS_ARDUINO_TOOLS_RECOVERY_LINK "error-recovery-link"
#define GOS_ARDUINO_TOOLS_RECOVERY_PROTOCOL "error-recovery-protocol"

#define GOS_ARDT_MOD_SET_MANUAL "set-manual"
#define GOS_ARDT_MOD_SET_SETPOINT "set-setpoint"
#define GOS_ARDT_MOD_SET_KP "set-kp"
#define GOS_ARDT_MOD_SET_KI "set-ki"
#define GOS_ARDT_MOD_SET_KD "set-kd"
#define GOS_ARDT_MOD_SET_TI "set-ti"
#define GOS_ARDT_MOD_SET_TD "set-td"

#define GOS_ARDT_MOD_GET_MANUAL "get-manual"
#define GOS_ARDT_MOD_GET_SETPOINT "get-setpoint"
#define GOS_ARDT_MOD_GET_KP "get-kp"
#define GOS_ARDT_MOD_GET_KI "get-ki"
#define GOS_ARDT_MOD_GET_KD "get-kd"
#define GOS_ARDT_MOD_GET_TI "get-ti"
#define GOS_ARDT_MOD_GET_TD "get-td"

#define MODBUS_SUCCESS 0

namespace po = ::boost::program_options;
namespace bl = boost::log;
namespace blk = boost::log::keywords;

namespace gat = ::gos::arduino::tools;
namespace gatt = ::gos::arduino::tools::text;
namespace gats = ::gos::arduino::tools::setting;
namespace gato = ::gos::arduino::tools::options;

static std::atomic_bool go;

#if defined(WIN32) && defined(GOS_NOT_USED)
/*
 * See Handling Ctrl+C in Windows Console Application
 * https://asawicki.info/news_1465_handling_ctrlc_in_windows_console_application
 *
 */
static BOOL WINAPI console_ctrl_handler(DWORD dwCtrlType)
{
  if (gats::isverbose()) {
    std::cout << "Stopping from console control handler" << std::endl;
  }
  go.store(false);
  return TRUE;
}
#endif

namespace initialize {
void logging();
}


namespace count {
const int coil = GOS_ARDT_COILS;
#ifdef GOS_ARDT_DISCRETE
const int discrete = GOS_ARDT_DISCRETE;
#endif
const int holding = GOS_ARDT_HOLDING;
const int input = GOS_ARDT_INPUT;
}

namespace set {
bool manual = false;
bool setpoint = false;
bool kp = false;
bool ki = false;
bool kd = false;
bool ti = false;
bool td = false;
}

namespace get {
bool manual = false;
bool setpoint = false;
bool kp = false;
bool ki = false;
bool kd = false;
bool ti = false;
bool td = false;
}

namespace read {
uint16_t manual = 0;
float setpoint = 0.0F, kp = 0.0F, ki = 0.0F, kd = 0.0F, ti = 0.0F, td = 0.0F;
}

namespace write {
uint16_t manual = 0;
float setpoint = 0.0F, kp = 0.0F, ki = 0.0F, kd = 0.0F, ti = 0.0F, td = 0.0F;
}

int main(int argc, char* argv[]) {

  modbus_error_recovery_mode recovery = MODBUS_ERROR_RECOVERY_NONE;

  errno_t errornumber;
  modbus_t* modbus = nullptr;
  int result;
  uint16_t output;
  float sensor;
  uint32_t timeout, utimeout;
  std::string name;
  bool modbusdebug = false;

  initialize::logging();

  uint8_t coils[GOS_ARDT_COILS];
#ifdef GOS_ARDT_DISCRETE
  uint8_t discrete[GOS_ARDT_DISCRETE];
#endif
  uint16_t registers[GOS_ARDT_HOLDING];
  uint16_t registersout[GOS_ARDT_HOLDING];
  uint16_t input[GOS_ARDT_INPUT];

#if defined(WIN32) && defined(GOS_NOT_USED)
  ::SetConsoleCtrlHandler(console_ctrl_handler, TRUE);
#endif
  gats::create();
  try {
    po::options_description optdescript(gato::general::Name);
    gato::general::create(optdescript);

    po::options_description communicationdescript(gato::communication::Name);
    gato::communication::create(communicationdescript);

    po::options_description timer(gato::timer::Name);
    gato::timer::create(
      timer,
      GOS_ARDUINO_TOOLS_MASTER_INTERVAL);

    po::options_description custom(GOS_ARDUINO_TOOLS_MASTER_NAME);
    custom.add_options()
      (GOS_ARDT_MOD_GET_MANUAL, "Get manual")
      (GOS_ARDT_MOD_GET_SETPOINT, "Get setpoint")
      (GOS_ARDT_MOD_GET_KP, "Get Kp")
      (GOS_ARDT_MOD_GET_KI, "Get Ki")
      (GOS_ARDT_MOD_GET_KD, "Get Kd")
      (GOS_ARDT_MOD_GET_TI, "Get TI")
      (GOS_ARDT_MOD_GET_TD, "Get Td")
      (GOS_ARDT_MOD_SET_MANUAL, po::value(&write::manual), "Set manual")
      (GOS_ARDT_MOD_SET_SETPOINT, po::value(&write::setpoint), "Set setpoint")
      (GOS_ARDT_MOD_SET_KP, po::value(&write::kp), "Set Kp")
      (GOS_ARDT_MOD_SET_KI, po::value(&write::ki), "Set Ki")
      (GOS_ARDT_MOD_SET_KD, po::value(&write::kd), "Set Kd")
      (GOS_ARDT_MOD_SET_TI, po::value(&write::ti), "Set TI")
      (GOS_ARDT_MOD_SET_TD, po::value(&write::td), "Set Td")
      (GOS_ARDUINO_TOOLS_RECOVERY_LINK, "Modbus link error recovery")
      (GOS_ARDUINO_TOOLS_RECOVERY_PROTOCOL, "Modbus protocol error recovery")
      (gatt::option::composite::Debug, gatt::option::description::Debug);

#ifdef GOS_NOT_YET_USED
    po::options_description tool(gato::tool::Name);
    gato::tool::create(tool);
#endif

    po::options_description clioptions(
      GOS_ARDUINO_TOOLS_MASTER_NAME " " GOST_USAGE, GOS_PO_LINE_LENGTH);
    clioptions
      .add(optdescript)
      .add(communicationdescript)
      .add(timer)
      .add(custom);

    po::variables_map varmap;

    /* First round with CLI parser only */
    po::command_line_parser cmdlinparser(argc, argv);
    po::parsed_options parseopt = cmdlinparser.options(clioptions)
      .style(po::command_line_style::default_style).run();
    po::store(parseopt, varmap);

    if (gato::handling::help(clioptions, varmap)) {
      return GOS_PO_EXIT_HELP;
    }

    std::string version = "1.0.0.0";
    std::string name = GOS_ARDUINO_TOOLS_MASTER_NAME;

    if (gato::handling::version(varmap, name, version)) {
      return GOS_PO_EXIT_VERSION;
    }

    gato::handling::verbosity(varmap);

    /* second round */
    po::notify(varmap);

    /* Create the configuration file options description */
    po::options_description configfdesc;
    configfdesc
      .add(communicationdescript)
      .add(timer)
      .add(custom);
    if (gato::handling::file(configfdesc, varmap) == EXIT_FAILURE) {
      return EXIT_FAILURE;
    }

    gato::handling::communication(varmap);
#ifdef GOS_NOT_YET_USED
    gato::handling::tool(varmap);
#endif

    if (varmap.count(gatt::Debug) > 0) {
      modbusdebug = true;
    }

    if (varmap.count(GOS_ARDUINO_TOOLS_RECOVERY_LINK) > 0) {
      recovery = MODBUS_ERROR_RECOVERY_LINK;
    } else if (varmap.count(GOS_ARDUINO_TOOLS_RECOVERY_PROTOCOL) > 0) {
      recovery = MODBUS_ERROR_RECOVERY_PROTOCOL;
    }

    set::manual = varmap.count(GOS_ARDT_MOD_SET_MANUAL) > 0;
    set::setpoint = varmap.count(GOS_ARDT_MOD_SET_SETPOINT) > 0;
    set::kp = varmap.count(GOS_ARDT_MOD_SET_KP) > 0;
    set::ki = varmap.count(GOS_ARDT_MOD_SET_KI) > 0;
    set::kd = varmap.count(GOS_ARDT_MOD_SET_KD) > 0;
    set::ti = varmap.count(GOS_ARDT_MOD_SET_TI) > 0;
    set::td = varmap.count(GOS_ARDT_MOD_SET_TD) > 0;

    get::manual = varmap.count(GOS_ARDT_MOD_GET_MANUAL) > 0;
    get::setpoint = varmap.count(GOS_ARDT_MOD_GET_SETPOINT) > 0;
    get::kp = varmap.count(GOS_ARDT_MOD_GET_KP) > 0;
    get::ki = varmap.count(GOS_ARDT_MOD_GET_KI) > 0;
    get::kd = varmap.count(GOS_ARDT_MOD_GET_KD) > 0;
    get::ti = varmap.count(GOS_ARDT_MOD_GET_TI) > 0;
    get::td = varmap.count(GOS_ARDT_MOD_GET_TD) > 0;
  }
  catch (::gos::arduino::tools::exception & er) {
    std::cerr << "Tool exception: "
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

  if (set::manual && set::setpoint) {
    std::cerr << "It is not possible to set both "
      "manual and setpoint at the same time" << std::endl;
    return EXIT_FAILURE;
  }

  if (gats::isnormal()) {
    std::cout << "Modbus Master tool" << std::endl;
    std::cout << "Use -h or --help for option help" << std::endl;
#if defined(WIN32) && defined(GOS_NOT_USED)
    std::cout << " or press break";
#endif
    std::cout << std::endl;
  }

  std::regex comgtnine("COM\\d{2,}");
  //  std::smatch smatch;
  if (std::regex_match(gats::communication::serialport, comgtnine)) {
    std::cout << "Converting the serial port '"
      << gats::communication::serialport << "'";
    gats::communication::serialport = "\\\\.\\" +
      gats::communication::serialport;
    std::cout << " to '" << gats::communication::serialport << "'" << std::endl;
  }

  auto handelreaderror = [&](const errno_t& errornumber)->void {
    switch (errornumber) {
    case ETIMEDOUT:
      std::cerr << "Failed to read registers from slave (" << errornumber
        << ") " << strerror(errornumber) << std::endl;
      break;
    case EMBMDATA:
      std::cerr << "To many registers requested from slave (" << errornumber
        << ") " << modbus_strerror(errornumber) << std::endl;
      break;
    default:
      std::cerr << "Failed to read registers from slave (" << errornumber
        << ") " << modbus_strerror(errornumber) << std::endl;
      break;
    }
  };
  auto handelwriteerror = [&](const errno_t& errornumber)->void {
    switch (errornumber) {
    case ETIMEDOUT:
      std::cerr << "Failed to write registers to slave (" << errornumber
        << ") " << strerror(errornumber) << std::endl;
      break;
    case EMBMDATA:
      std::cerr << "To many registers requested for slave (" << errornumber
        << ") " << modbus_strerror(errornumber) << std::endl;
      break;
    default:
      std::cerr << "Failed to write registers from slave (" << errornumber
        << ") " << modbus_strerror(errornumber) << std::endl;
      break;
    }
  };

  modbus = modbus_new_rtu(
    gats::communication::serialport.c_str(),
    gats::communication::serialbaud,
    'N', 8, 1);
  if (modbus != nullptr) {
    std::cout << "Modbus RTU master" << std::endl;
  } else {
    std::cerr << "Out of memory for Modbus RTU" << std::endl;
    goto gos_exit_with_failure;
  }
  result = modbus_set_slave(modbus, gats::communication::slave_id);
  if (result == MODBUS_SUCCESS) {
    std::cout << "Setting Modbus slave ID to "
      << gats::communication::slave_id << std::endl;
  } else {
    errornumber = errno;
    std::cerr << "Failed to modify Modbus slave ID to "
      << gats::communication::slave_id << " ("
      << errornumber << ") " << modbus_strerror(errornumber) << std::endl;
    goto gos_exit_with_failure;
  }

  if (modbusdebug) {
    result = modbus_set_debug(modbus, TRUE);
    if (result == MODBUS_SUCCESS) {
      std::cout << "Debug is turned on" << std::endl;
    } else {
      errornumber = errno;
      std::cerr << "Failed to Modbus in debug ("
        << errornumber << ") " << modbus_strerror(errornumber) << std::endl;
      goto gos_exit_with_failure;
    }
  }
  result = modbus_set_error_recovery(modbus, recovery);
  if (result == MODBUS_SUCCESS) {
    std::cout << "Setting error recovery to " << recovery << std::endl;
  } else {
    errornumber = errno;
    std::cerr << "Failed to set error recovery to " << recovery << " ("
      << errornumber << ") " << modbus_strerror(errornumber) << std::endl;
    goto gos_exit_with_failure;
  }

  result = modbus_get_response_timeout(modbus, &timeout, &utimeout);
  if (result == MODBUS_SUCCESS) {
    std::cout << "The response timeout is " << timeout
      << " and " << utimeout << std::endl;
  } else {
    errornumber = errno;
    std::cerr << "Failed to get response timeout ("
      << errornumber << ") " << modbus_strerror(errornumber) << std::endl;
    goto gos_exit_with_failure;
  }

  result = modbus_connect(modbus);
  if (result == MODBUS_SUCCESS) {
    std::cout << "Successfully connected to " << gats::communication::serialport
      << " at " << gats::communication::serialbaud << std::endl;
  } else {
    errornumber = errno;
    std::cerr << "Failed to connect to " << gats::communication::serialport
      << " at " << gats::communication::serialbaud << " (" << errornumber
      << ") " << modbus_strerror(errornumber) << std::endl;
    goto gos_exit_with_failure;
  }

  result = modbus_read_input_registers(modbus, 0, 3, input);
  if (result >= 0) {
    output = input[0];
    sensor = GOS_ARDT_GET_FL(input + 1);
    std::cout << "Output is " << output
      << " and sensor is " << sensor << std::endl;
  } else {
    errornumber = errno;
    std::cerr << "Failed to read input registers from slave ("
      << errornumber << ") " << modbus_strerror(errornumber) << std::endl;
  }

  if (get::manual) {
    result = modbus_read_registers(modbus, 0, 1, &read::manual);
    if (result >= 0) {
      std::cout << "Manual is " << read::manual << std::endl;
    } else {
      handelreaderror(errno);
    }
  }

  if (get::setpoint) {
    result = modbus_read_registers(modbus, 1, 2, registers);
    if (result >= 0) {
      read::setpoint = GOS_ARDT_GET_FL(registers);
      std::cout << "Setpoint is " << read::setpoint << std::endl;
    } else {
      handelreaderror(errno);
    }
  }
  if (get::kp) {
    result = modbus_read_registers(modbus, 3, 2, registers);
    if (result >= 0) {
      read::kp = GOS_ARDT_GET_FL(registers);
      std::cout << "Kp is " << read::kp << std::endl;
    } else {
      handelreaderror(errno);
    }
  }
  if (get::ki) {
    result = modbus_read_registers(modbus, 5, 2, registers);
    if (result >= 0) {
      read::ki = GOS_ARDT_GET_FL(registers);
      std::cout << "Ki is " << read::ki << std::endl;
    } else {
      handelreaderror(errno);
    }
  }
  if (get::kd) {
    result = modbus_read_registers(modbus, 7, 2, registers);
    if (result >= 0) {
      read::kd = GOS_ARDT_GET_FL(registers);
      std::cout << "Kd is " << read::kd << std::endl;
    } else {
      handelreaderror(errno);
    }
  }
  if (get::ti) {
    result = modbus_read_registers(modbus, 9, 2, registers);
    if (result >= 0) {
      read::ti = GOS_ARDT_GET_FL(registers);
      std::cout << "Ti is " << read::ti << std::endl;
    } else {
      handelreaderror(errno);
    }
  }
  if (get::td) {
    result = modbus_read_registers(modbus, 11, 2, registers);
    if (result >= 0) {
      read::td = GOS_ARDT_GET_FL(registers);
      std::cout << "Td is " << read::td << std::endl;
    } else {
      handelreaderror(errno);
    }
  }

  if (set::manual) {
    result = modbus_write_register(modbus, 0, write::manual);
    if (result >= 0) {
      std::cout << "Writing manual as " << write::manual << std::endl;
    } else {
      handelwriteerror(errno);
    }
  }
  if (set::setpoint) {
    GOS_ARDT_SET_FL(write::setpoint, registers);
    result = modbus_write_registers(modbus, 1, 2, registers);
    if (result >= 0) {
      std::cout << "Writing setpoint as " << write::setpoint << std::endl;
    } else {
      handelwriteerror(errno);
    }
  }
  if (set::kp) {
    GOS_ARDT_SET_FL(write::kp, registers);
    result = modbus_write_registers(modbus, 3, 2, registers);
    if (result >= 0) {
      std::cout << "Writing kp as " << write::kp << std::endl;
    } else {
      handelwriteerror(errno);
    }
  }
  if (set::ki) {
    GOS_ARDT_SET_FL(write::ki, registers);
    result = modbus_write_registers(modbus, 5, 2, registers);
    if (result >= 0) {
      std::cout << "Writing ki as " << write::ki << std::endl;
    } else {
      handelwriteerror(errno);
    }
  }
  if (set::kd) {
    GOS_ARDT_SET_FL(write::kd, registers);
    result = modbus_write_registers(modbus, 7, 2, registers);
    if (result >= 0) {
      std::cout << "Writing kd as " << write::kd << std::endl;
    } else {
      handelwriteerror(errno);
    }
  }
  if (set::ti) {
    GOS_ARDT_SET_FL(write::ti, registers);
    result = modbus_write_registers(modbus, 9, 2, registers);
    if (result >= 0) {
      std::cout << "Writing ti as " << write::ti << std::endl;
    } else {
      handelwriteerror(errno);
    }
  }
  if (set::td) {
    GOS_ARDT_SET_FL(write::td, registers);
    result = modbus_write_registers(modbus, 11, 2, registers);
    if (result >= 0) {
      std::cout << "Writing td as " << write::td << std::endl;
    } else {
      handelwriteerror(errno);
    }
  }


  std::cout << "Closing and freeing Modbus" << std::endl;
  modbus_close(modbus);
  modbus_free(modbus);

  return EXIT_SUCCESS;

gos_exit_with_failure:
  if (modbus != nullptr) {
    modbus_close(modbus);
    modbus_free(modbus);
    modbus = nullptr;
  }
  return EXIT_FAILURE;
}


//static ::boost::shared_ptr<> logfile;

namespace initialize {
void logging() {
  bl::register_simple_formatter_factory<bl::trivial::severity_level, char>(
    "Severity");

  auto logfile = bl::add_file_log(
    blk::file_name = GOS_ARDUINO_TOOLS_MASTER_LOG_FILE,
    blk::format = "[%TimeStamp%] [%ThreadID%] [%Severity%] %Message%");

  bl::core::get()->set_filter(
    bl::trivial::severity >= bl::trivial::debug);

  bl::add_common_attributes();
}
}

