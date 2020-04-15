#include <cassert>
#include <cstring>

#include <iostream>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>

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

#include <gos/arduino/tools/pid/modbus/master.h>

#define GOS_ARDUINO_TOOLS_MASTER_NAME "master"

#define GOS_ARDT_MOD_MANUAL "manual"
#define GOS_ARDT_MOD_SETPOINT "setpoint"
#define GOS_ARDT_MOD_KP "kp"
#define GOS_ARDT_MOD_KI "ki"
#define GOS_ARDT_MOD_KD "kd"
#define GOS_ARDT_MOD_INTERNAL "internal"
#define GOS_ARDT_MOD_FORCE "force"
#define GOS_ARDT_MOD_FINAL "final"

#define GOS_ARDUINO_TOOLS_MASTER_DEFAULT_INTERVAL 1000

namespace po = ::boost::program_options;

namespace gat = ::gos::arduino::tools;
namespace gatt = ::gos::arduino::tools::text;
namespace gats = ::gos::arduino::tools::setting;
namespace gato = ::gos::arduino::tools::options;

namespace gatp = ::gos::arduino::tools::pid;
namespace gatpm = ::gos::arduino::tools::pid::modbus;

typedef std::chrono::steady_clock Clock;
typedef Clock::time_point Time;
typedef Clock::duration Duration;

static std::atomic_bool go;

static HANDLE handle = NULL;

#if defined(_WIN32)
/*
 * See Handling Ctrl+C in Windows Console Application
 * https://asawicki.info/news_1465_handling_ctrlc_in_windows_console_application
 *
 */
static BOOL WINAPI console_ctrl_handler(DWORD dwCtrlType) {
  if (gats::isverbose()) {
    std::cerr << "Stopping from console control handler" << std::endl;
  }
  go.store(false);
  if (handle) {
    SetEvent(handle);
  }
  return TRUE;
}
#endif

int main(int argc, char* argv[]) {
  go.store(true);
  int final = -1;
  int retval = EXIT_SUCCESS;
  gatpm::types::result result;
#if defined(_WIN32)
  ::SetConsoleCtrlHandler(console_ctrl_handler, TRUE);
#endif
  gats::create();
  try {
    po::options_description optdescript(gato::general::Name);
    gato::general::create(optdescript);

    po::options_description communicationdescript(gato::communication::Name);
    gato::communication::create(communicationdescript);

    po::options_description timer(gato::timer::Name);
    gato::timer::create(timer, GOS_ARDUINO_TOOLS_MASTER_DEFAULT_INTERVAL);

    po::options_description custom(GOS_ARDUINO_TOOLS_MASTER_NAME);
    custom.add_options()
      (GOS_ARDT_MOD_MANUAL, po::value<gatp::types::Unsigned>(), "manual value")
      (GOS_ARDT_MOD_FORCE, po::value<gatp::types::Unsigned>(), "force")
      (GOS_ARDT_MOD_FINAL, po::value<gatp::types::Unsigned>(), "final force")
      (GOS_ARDT_MOD_SETPOINT, po::value<gatp::types::Real>(), "setpoint")
      (GOS_ARDT_MOD_KP, po::value<gatp::types::Real>(), "kp")
      (GOS_ARDT_MOD_KI, po::value<gatp::types::Real>(), "ki")
      (GOS_ARDT_MOD_KD, po::value<gatp::types::Real>(), "kd")
      (GOS_ARDT_MOD_INTERNAL, GOS_ARDT_MOD_INTERNAL);

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

    result = gatpm::master::initialize(
      gats::communication::serial::port.c_str(),
      gats::communication::serial::baud,
      gats::slave::id);

    if (result != gatpm::types::result::success) {
      std::cerr << gatpm::master::report::error::last() << std::endl;
      return EXIT_FAILURE;
    }

    result = gatpm::master::connect();
    if(result != gatpm::types::result::success) {
      std::cerr << "Failed to connect to Modbus Slave " << gats::slave::id
        << " through " << gats::communication::serial::port
        << " baud rate " << gats::communication::serial::baud << std::endl;
      goto gos_arduino_tools_pid_modbus_master_exit_failure;
    }

    result = gatpm::master::write::interval(
      gats::timing::interval::milliseconds::loop);
    if (result != gatpm::types::result::success) {
      std::cerr << "Failed to write interval" << std::endl;
      goto gos_arduino_tools_pid_modbus_master_exit_failure;
    }

    if (
      varmap.count(GOS_ARDT_MOD_KP) > 0 &&
      varmap.count(GOS_ARDT_MOD_KI) > 0 &&
      varmap.count(GOS_ARDT_MOD_KD) > 0) {
      result = gatpm::master::write::tuning(
        varmap[GOS_ARDT_MOD_KP].as<gatp::types::Real>(),
        varmap[GOS_ARDT_MOD_KI].as<gatp::types::Real>(),
        varmap[GOS_ARDT_MOD_KD].as<gatp::types::Real>());
      if (result != gatpm::types::result::success) {
        std::cerr << gatpm::master::report::error::last() << std::endl;
        goto gos_arduino_tools_pid_modbus_master_exit_failure;
      }
    } else {
      if (varmap.count(GOS_ARDT_MOD_KP) > 0) {
        result = gatpm::master::write::kp(
          varmap[GOS_ARDT_MOD_KP].as<gatp::types::Real>());
        if (result != gatpm::types::result::success) {
          std::cerr << gatpm::master::report::error::last() << std::endl;
          goto gos_arduino_tools_pid_modbus_master_exit_failure;
        }
      }
      if (varmap.count(GOS_ARDT_MOD_KI) > 0) {
        result = gatpm::master::write::ki(
          varmap[GOS_ARDT_MOD_KI].as<gatp::types::Real>());
        if (result != gatpm::types::result::success) {
          std::cerr << gatpm::master::report::error::last() << std::endl;
          goto gos_arduino_tools_pid_modbus_master_exit_failure;
        }
      }
      if (varmap.count(GOS_ARDT_MOD_KD) > 0) {
        result = gatpm::master::write::kd(
          varmap[GOS_ARDT_MOD_KD].as<gatp::types::Real>());
        if (result != gatpm::types::result::success) {
          std::cerr << gatpm::master::report::error::last() << std::endl;
          goto gos_arduino_tools_pid_modbus_master_exit_failure;
        }
      }
    }

    if (varmap.count(GOS_ARDT_MOD_MANUAL) > 0) {
      result = gatpm::master::write::manual(
        varmap[GOS_ARDT_MOD_MANUAL].as<gatp::types::Unsigned>());
      if (result != gatpm::types::result::success) {
        std::cerr << gatpm::master::report::error::last() << std::endl;
        goto gos_arduino_tools_pid_modbus_master_exit_failure;
      }
    }

    if (varmap.count(GOS_ARDT_MOD_SETPOINT) > 0) {
      result = gatpm::master::write::setpoint(
        varmap[GOS_ARDT_MOD_SETPOINT].as<gatp::types::Real>());
      if (result != gatpm::types::result::success) {
        std::cerr << gatpm::master::report::error::last() << std::endl;
        goto gos_arduino_tools_pid_modbus_master_exit_failure;
      }
    }

    if (varmap.count(GOS_ARDT_MOD_FINAL) > 0) {
      final = static_cast<int>(
        varmap[GOS_ARDT_MOD_FINAL].as<gatp::types::Unsigned>());
    }

    if (varmap.count(GOS_ARDT_MOD_FORCE) > 0) {
      result = gatpm::master::write::force(
        varmap[GOS_ARDT_MOD_FORCE].as<gatp::types::Unsigned>());
      if (result != gatpm::types::result::success) {
        std::cerr << gatpm::master::report::error::last() << std::endl;
        goto gos_arduino_tools_pid_modbus_master_exit_failure;
      }
    }

    /* Create the header */
    std::cout << "time,status";
    if (varmap.count(GOS_ARDT_MOD_KP)) {
      std::cout << ",kp";
    }
    if (varmap.count(GOS_ARDT_MOD_KI)) {
      std::cout << ",ki";
    }
    if (varmap.count(GOS_ARDT_MOD_KD)) {
      std::cout << ",kd";
    }
    if (varmap.count(GOS_ARDT_MOD_MANUAL)) {
      std::cout << ",manual";
    }
    if (varmap.count(GOS_ARDT_MOD_SETPOINT)) {
      std::cout << ",setpoint";
    }
    std::cout << ",output,temperature";
    if (varmap.count(GOS_ARDT_MOD_INTERNAL)) {
      std::cout << ",error,integral,derivative";
    }
    std::cout << std::endl;

    gatp::types::registry::Holding holding;
    gatp::types::registry::Input input;

    for (int rt = 0; rt < 3; ++rt) {
      result = gatpm::master::read::holding(holding);
      if (result == gatpm::types::result::success) {
        break;
      } else {
        std::cerr << gatpm::master::report::error::last() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
      }
    }
    if (result != gatpm::types::result::success) {
      goto gos_arduino_tools_pid_modbus_master_exit_failure;
    }

    DWORD wait;
    Duration duration;
    std::chrono::milliseconds dms;
    Time time, starttime = Clock::now();
    bool localgo = go.load();
    while (localgo) {
      time = Clock::now();
      duration = time - starttime;
      dms = std::chrono::duration_cast<std::chrono::milliseconds>(duration);
      std::cout << static_cast<double>(dms.count()) / 1000.0;
      result = gatpm::master::read::input(input);
      if (result == gatpm::types::result::success) {
        std::cout << "," << input.Status;
      } else {
        std::cout << "," << -1;
        std::cerr << gatpm::master::report::error::last() << std::endl;
      }
      if (varmap.count(GOS_ARDT_MOD_KP)) {
        std::cout << "," << holding.Kp;
      }
      if (varmap.count(GOS_ARDT_MOD_KI)) {
        std::cout << "," << holding.Ki;
      }
      if (varmap.count(GOS_ARDT_MOD_KD)) {
        std::cout << "," << holding.Kd;
      }
      if (varmap.count(GOS_ARDT_MOD_MANUAL)) {
        std::cout << "," << holding.Manual;
      }
      if (varmap.count(GOS_ARDT_MOD_SETPOINT)) {
        std::cout << "," << holding.Setpoint;
      }

      if (result == gatpm::types::result::success) {
        std::cout << "," << input.Output << "," << input.Temperature;
        if (varmap.count(GOS_ARDT_MOD_INTERNAL)) {
          std::cout << "," << input.Error
            << "," << input.Integral
            << "," << input.Derivative;
        }
      } else {
        std::cout << ",,";
        if (varmap.count(GOS_ARDT_MOD_INTERNAL)) {
          std::cout << ",,,";
        }
      }

      std::cout << std::endl;

      if (localgo = go.load()) {
        if (handle) {
          CloseHandle(handle);
        }
        handle = CreateEvent(
          NULL,               // Default security attributes
          TRUE,               // Manual-reset event
          FALSE,              // Initial state is non-signaled
          NULL                // Object name
        );
        if (handle) {
          wait = WaitForSingleObject(
            handle,
            gats::timing::interval::milliseconds::loop);
          switch (wait) {
          case WAIT_OBJECT_0:
          case WAIT_TIMEOUT:
            break;
          case WAIT_ABANDONED:
          case WAIT_FAILED:
          default:
            std::cerr << "Waiting for the go lock failed" << std::endl;
            goto gos_arduino_tools_pid_modbus_master_exit_failure;
          }
        } else {
          std::cerr << "Failed to create loop wait event" << std::endl;
          goto gos_arduino_tools_pid_modbus_master_exit_failure;
        }
      }
    }
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

  goto gos_arduino_tools_pid_modbus_master_exit;

gos_arduino_tools_pid_modbus_master_exit_failure:
  retval = EXIT_FAILURE;

gos_arduino_tools_pid_modbus_master_exit:
  if (final >= 0) {
    gatpm::master::write::force(static_cast<gatp::types::Unsigned>(final));
  }
  gatpm::master::disconnect();
  gatpm::master::shutdown();
  return retval;
}
