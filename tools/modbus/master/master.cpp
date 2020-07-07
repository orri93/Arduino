#include <cassert>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>

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
#include <boost/optional.hpp>

#include <modbus.h>

#include <gos/arduino/tools/text.h>
#include <gos/arduino/tools/options.h>
#include <gos/arduino/tools/setting.h>
#include <gos/arduino/tools/exception.h>
#include <gos/arduino/tools/statistics.h>
#include <gos/arduino/tools/window.h>

#include <gos/arduino/tools/pid/modbus/master.h>

#include <gos/arduino/tools/pid/tuning/blackbox.h>

#define GOS_ARDUINO_TOOLS_MASTER_NAME "master"

#define GOS_ARDUINO_TOOLS_MASTER_BASE_LINE_ADDITION 2.5F

#define GOS_ARDT_MOD_MANUAL "manual"
#define GOS_ARDT_MOD_SETPOINT "setpoint"
#define GOS_ARDT_MOD_KP "kp"
#define GOS_ARDT_MOD_KI "ki"
#define GOS_ARDT_MOD_KD "kd"
#define GOS_ARDT_MOD_SD "sd"
#define GOS_ARDT_MOD_MIN_KP "min-kp"
#define GOS_ARDT_MOD_MAX_KP "max-kp"
#define GOS_ARDT_MOD_MIN_KI "min-ki"
#define GOS_ARDT_MOD_MAX_KI "max-ki"
#define GOS_ARDT_MOD_OUTPUT "output"
#define GOS_ARDT_MOD_OUTPUT_O "output,o"
#define GOS_ARDT_MOD_INTERNAL "internal"
#define GOS_ARDT_MOD_FORCE "force"
#define GOS_ARDT_MOD_FINAL "final"
#define GOS_ARDT_MOD_WINDOW "window"
#define GOS_ARDT_MOD_STABLE_DURATION "stable-duration"
#define GOS_ARDT_MOD_BASE_LINE "base-line"
#define GOS_ARDT_MOD_SEPARATOR "separator"
#define GOS_ARDT_MOD_TUNING_OUTPUT "tuning-output"
#define GOS_ARDT_MOD_TUNING "tuning"
#define GOS_ARDT_MOD_TUNING_MODE_BLACK_BOX "black"

#define GOS_ARDT_DEFAULT_TUNING_OUTPUT "tuning"

#define GOS_ARDT_DEFAULT_SEPARATOR ","

#define GOS_ARDT_DEFAULT_WINDOW 15
#define GOS_ARDT_DEFAULT_STABLE_DURATION 15

#define GOS_ARDT_DEFAULT_SD 0.125F

#define GOS_ARDT_DEFAULT_MIN_KP 1.0F
#define GOS_ARDT_DEFAULT_MAX_KP 5.0F
#define GOS_ARDT_DEFAULT_MIN_KI 0.001F
#define GOS_ARDT_DEFAULT_MAX_KI 0.01F


#define GOS_ARDUINO_TOOLS_MASTER_DEFAULT_INTERVAL 1000
#define GOS_ARDUINO_TOOLS_MASTER_DEFAULT_RETRY 3

#define GOS_SUCCESS EXIT_SUCCESS
#define GOS_FAILURE EXIT_FAILURE

namespace po = ::boost::program_options;

namespace gat = ::gos::arduino::tools;
namespace gatt = ::gos::arduino::tools::text;
namespace gats = ::gos::arduino::tools::setting;
namespace gato = ::gos::arduino::tools::options;

namespace gatp = ::gos::arduino::tools::pid;
namespace gatpm = ::gos::arduino::tools::pid::modbus;
namespace gatpmt = ::gos::arduino::tools::pid::modbus::types;
namespace gatpt = ::gos::arduino::tools::pid::types;

typedef ::std::chrono::steady_clock Clock;
typedef Clock::time_point Time;
typedef Clock::duration Duration;

typedef ::boost::optional<gatp::types::Real> OptionalReal;
typedef ::boost::optional<gatp::types::Unsigned> OptionalUnsigned;

typedef gat::Statistics<gatp::types::Real> Statistics;
typedef gat::statistics::window<gatp::types::Real> Window;

typedef ::std::unique_ptr<std::ofstream> OutputStreamPointer;

enum class TuningMode {
  undefined,
  blackbox
};

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

namespace gos {
namespace arduino {
namespace tools {
namespace modbus {
namespace master {

const gat::pid::types::Real PeakFactor = 1.0F;
const gat::pid::types::Real StableFactor = 1.0F;

struct Initialized {
  OptionalReal Kp;
  OptionalReal Ki;
  OptionalReal Kd;
  OptionalReal Setpoint;
  OptionalUnsigned Manual;
};

struct Evaluation {
  gat::pid::types::Real Kp;
  gat::pid::types::Real Ki;
  gat::pid::types::Real Kd;
  gat::pid::types::Real PeakError;
  gat::pid::types::Real StableError;
  gat::pid::types::Real Performance;
};

typedef std::unique_ptr<gat::modbus::master::Evaluation> EvaluationPointer;

static gat::pid::types::Real evaluate(
  gat::pid::types::Real peakerror,
  gat::pid::types::Real stableerror);

static void evaluate(Evaluation& evaluation);
static void replace(EvaluationPointer& last, const Evaluation& current);

namespace read {
static int holding(gatpt::registry::Holding& holding, const int& maxretry = 
  GOS_ARDUINO_TOOLS_MASTER_DEFAULT_RETRY);
static int input(gatpt::registry::Input& input, const int& maxretry = 
  GOS_ARDUINO_TOOLS_MASTER_DEFAULT_RETRY);
} // namespace read

namespace write {
static bool tuning(
  const gatpt::Real& kp,
  const gatpt::Real& ki,
  const gatpt::Real& kd,
  const int& maxretry = GOS_ARDUINO_TOOLS_MASTER_DEFAULT_RETRY);
static bool tuning(
  const gatpt::Real& kp,
  const gatpt::Real& ki,
  const int& maxretry = GOS_ARDUINO_TOOLS_MASTER_DEFAULT_RETRY);
static bool kp(const gatpt::Real& kp, const int& maxretry =
  GOS_ARDUINO_TOOLS_MASTER_DEFAULT_RETRY);
static bool ki(const gatpt::Real& ki, const int& maxretry =
  GOS_ARDUINO_TOOLS_MASTER_DEFAULT_RETRY);
static bool kd(const gatpt::Real& kd, const int& maxretry =
  GOS_ARDUINO_TOOLS_MASTER_DEFAULT_RETRY);
static bool setpoint(const gatpt::Real& setpoint, const int& maxretry =
  GOS_ARDUINO_TOOLS_MASTER_DEFAULT_RETRY);
static bool manual(const gatpt::Unsigned& manual, const int& maxretry =
  GOS_ARDUINO_TOOLS_MASTER_DEFAULT_RETRY);
static bool force(const gatpt::Unsigned& force, const int& maxretry =
  GOS_ARDUINO_TOOLS_MASTER_DEFAULT_RETRY);
static bool interval(const gatpt::Unsigned& interval, const int& maxretry =
  GOS_ARDUINO_TOOLS_MASTER_DEFAULT_RETRY);
} // namespace write

namespace initial {
namespace apply {
static int tuning(
  gat::modbus::master::Initialized& initialized,
  po::variables_map& varmap,
  gatp::tuning::black::box::Variables& variables,
  const gatp::tuning::black::box::Parameters& parameters,
  const TuningMode& mode);
} // namespace master
} // namespace initial

} // namespace master
} // namespace modbus
} // namespace tools
} // namespace arduino
} // namespace gos

namespace gatm = ::gos::arduino::tools::modbus;
namespace gatmm = ::gos::arduino::tools::modbus::master;
namespace gatmmi = ::gos::arduino::tools::modbus::master::initial;

int main(int argc, char* argv[]) {
  go.store(true);
  int nresult;
  int round = 0;
  int retval = EXIT_SUCCESS;
  gatpm::types::result result;

  double elapsed;

  size_t windowsize;
  gatp::types::Unsigned stableduration;

  std::string outputfilepath, tuningoutputfilepath, separator;

  TuningMode tuningmode = TuningMode::undefined;

  OptionalReal baseline;
  OptionalUnsigned force, final;

  gatp::types::Real kd, setpoint;

  gat::modbus::master::Initialized initialized;

  gatp::types::registry::Holding holding;
  gatp::types::registry::Input input;

  gatp::tuning::black::box::Parameters parameters;
  gatp::tuning::black::box::Variables variables;

  gat::modbus::master::EvaluationPointer lastevaluation;

  OutputStreamPointer output, tuningoutput;

  Statistics statistics;
  Window window;

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
      (GOS_ARDT_MOD_TUNING, po::value<std::string>(),
        "tuning method (black is the only one currently supported)")
      (GOS_ARDT_MOD_SD, po::value(&(parameters.Sd))
        ->default_value(GOS_ARDT_DEFAULT_SD),
        "standard deviation for the black box tuning")
      (GOS_ARDT_MOD_MIN_KP, po::value(&(parameters.Kp.lowest))
        ->default_value(GOS_ARDT_DEFAULT_MIN_KP),
        "minimum Kp value for the black box tuning")
      (GOS_ARDT_MOD_MAX_KP, po::value(&(parameters.Kp.highest))
        ->default_value(GOS_ARDT_DEFAULT_MAX_KP),
        "minimum Kp value for the black box tuning")
      (GOS_ARDT_MOD_MIN_KI, po::value(&(parameters.Ki.lowest))
        ->default_value(GOS_ARDT_DEFAULT_MIN_KI),
        "minimum Ki value for the black box tuning")
      (GOS_ARDT_MOD_MAX_KI, po::value(&(parameters.Ki.highest))
        ->default_value(GOS_ARDT_DEFAULT_MAX_KI),
        "minimum Ki value for the black box tuning")
      (GOS_ARDT_MOD_BASE_LINE, po::value<gatp::types::Real>(),
        "base line for the black box tuning")
      (GOS_ARDT_MOD_OUTPUT_O, po::value<std::string>(),
        "output to file instead of standard out")
      (GOS_ARDT_MOD_TUNING_OUTPUT, po::value(&(tuningoutputfilepath))
        ->default_value(GOS_ARDT_DEFAULT_TUNING_OUTPUT),
        "tuning output file path")
      (GOS_ARDT_MOD_SEPARATOR, po::value(&(separator))
        ->default_value(GOS_ARDT_DEFAULT_SEPARATOR),
        "value separator")
      (GOS_ARDT_MOD_WINDOW, po::value(&(windowsize))
        ->default_value(GOS_ARDT_DEFAULT_WINDOW),
        "running window size for tuning")
      (GOS_ARDT_MOD_STABLE_DURATION, po::value(&(stableduration))
        ->default_value(GOS_ARDT_DEFAULT_STABLE_DURATION),
        "stable time for tuning")
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

    if (varmap.count(GOS_ARDT_MOD_BASE_LINE) > 0) {
      baseline = varmap[GOS_ARDT_MOD_BASE_LINE].as<gatp::types::Real>();
    }

    if (varmap.count(GOS_ARDT_MOD_TUNING) > 0) {
      std::string tuningmodetext =
        varmap[GOS_ARDT_MOD_TUNING].as<std::string>();
      if (tuningmodetext.compare(GOS_ARDT_MOD_TUNING_MODE_BLACK_BOX) == 0) {
        tuningmode = TuningMode::blackbox;
      } else {
        std::cerr << "'" << tuningmodetext
          << "' is not supported tune mode" << std::endl;
        return EXIT_FAILURE;
      }
    }

    window.set(windowsize);

    result = gatpm::master::initialize(
      gats::communication::serial::port.c_str(),
      gats::communication::serial::baud,
      gats::slave::id);

    if (result != gatpm::types::result::success) {
      std::cerr << gatpm::master::report::error::last() << std::endl;
      return EXIT_FAILURE;
    }

    result = gatpm::master::connect();
    if (result != gatpm::types::result::success) {
      std::cerr << "Failed to connect to Modbus Slave " << gats::slave::id
        << " through " << gats::communication::serial::port
        << " baud rate " << gats::communication::serial::baud << std::endl;
      goto gos_arduino_tools_pid_modbus_master_exit_failure;
    }

    if (!gat::modbus::master::write::interval(static_cast<
      gatp::types::Unsigned>(gats::timing::interval::milliseconds::loop))) {
      std::cerr << "Failed to write interval" << std::endl;
      goto gos_arduino_tools_pid_modbus_master_exit_failure;
    }

    std::streambuf* bufferpointer = nullptr;
    if (varmap.count(GOS_ARDT_MOD_OUTPUT) == 0) {
      bufferpointer = std::cout.rdbuf();
      if (bufferpointer == nullptr) {
        std::cerr << "Failed to get buffer pointer from standard out"
          << std::endl;
        goto gos_arduino_tools_pid_modbus_master_exit_failure;
      }
    } else {
      outputfilepath = varmap[GOS_ARDT_MOD_OUTPUT].as<std::string>();
      output = std::make_unique<std::ofstream>();
      if (output) {
        output->open(outputfilepath, std::ios::trunc | std::ios::out);
        bufferpointer = output->rdbuf();
        if (bufferpointer == nullptr) {
          std::cerr << "Failed to get buffer pointer from output file"
            << std::endl;
          goto gos_arduino_tools_pid_modbus_master_exit_failure;
        }
      } else {
        std::cerr << "Failed to create an output file" << std::endl;
        goto gos_arduino_tools_pid_modbus_master_exit_failure;
      }
    }
    assert(bufferpointer != nullptr);
    std::ostream pidoutput(bufferpointer);

    if (gat::modbus::master::initial::apply::tuning(
      initialized,
      varmap,
      variables,
      parameters,
      tuningmode) != GOS_SUCCESS) {
      goto gos_arduino_tools_pid_modbus_master_exit_failure;
    }

    if (varmap.count(GOS_ARDT_MOD_FINAL) > 0) {
      final = varmap[GOS_ARDT_MOD_FINAL].as<gatp::types::Unsigned>();
    }

    switch (tuningmode) {
    case TuningMode::blackbox:
      force = GOT_PI_FORCE_AUTO;
      break;
    default:
      if (varmap.count(GOS_ARDT_MOD_FORCE) > 0) {
        force = varmap[GOS_ARDT_MOD_FINAL].as<gatp::types::Unsigned>();
      }
      break;
    }

    if (force.has_value()) {
      if (!gat::modbus::master::write::force(force.get())) {
        goto gos_arduino_tools_pid_modbus_master_exit_failure;
      }
    }

    /* Create the header */
    pidoutput << "time";
    switch (tuningmode) {
    case TuningMode::blackbox:
      pidoutput << separator << "round";
      break;
    }
    pidoutput << separator << "status";
    switch (tuningmode) {
    case TuningMode::blackbox:
      pidoutput << separator << "kp";
      pidoutput << separator << "ki";
      break;
    default:
      if (varmap.count(GOS_ARDT_MOD_KP)) {
        pidoutput << separator << "kp";
      }
      if (varmap.count(GOS_ARDT_MOD_KI)) {
        pidoutput << separator << "ki";
      }
      break;
    }
    if (varmap.count(GOS_ARDT_MOD_KD)) {
      pidoutput << separator << "kd";
    }
    if (varmap.count(GOS_ARDT_MOD_MANUAL)) {
      pidoutput << separator << "manual";
    }
    if (varmap.count(GOS_ARDT_MOD_SETPOINT)) {
      pidoutput << separator << "setpoint";
    }
    pidoutput << separator << "output" << separator << "temperature";
    if (varmap.count(GOS_ARDT_MOD_INTERNAL)) {
      pidoutput
        << separator << "error"
        << separator << "integral"
        << separator << "derivative";
    }
    pidoutput << std::endl;

    if (gat::modbus::master::read::holding(holding) != GOS_SUCCESS) {
      goto gos_arduino_tools_pid_modbus_master_exit_failure;
    }

    switch (tuningmode) {
    case TuningMode::blackbox:
      if (initialized.Kd.has_value()) {
        kd = initialized.Kd.get();
      } else {
        kd = holding.Kd;
      }
      if (initialized.Setpoint.has_value()) {
        setpoint = initialized.Setpoint.get();
      } else {
        setpoint = holding.Setpoint;
      }
      tuningoutput = std::make_unique<std::ofstream>();
      tuningoutput->open(tuningoutputfilepath, std::ios::out | std::ios::trunc);

      (*tuningoutput) << "time" << separator << "round"
        << separator << "kp" << separator << "ki" << separator << "kd"
        << separator << "peak"
        << separator << "peak-error"
        << separator << "stable-error"
        << separator << "performance";
      if (varmap.count(GOS_ARDT_MOD_INTERNAL)) {
        (*tuningoutput)
          << separator << "over"
          << separator << "under"
          << separator << "stable"
          << separator << "cooling"
          << separator << "count";
      }
      (*tuningoutput) << std::endl;
      break;
    }

    DWORD wait;
    Duration duration;
    std::chrono::milliseconds dms;
    double
      coolingelapsed = 0.0,
      startroundelapsed = 0.0,
      stableelapsed = 0.0,
      overelapsed = 0.0,
      underelapsed = 0.0;
    Time time, stabletime, starttime = Clock::now();
    bool
      isover = false,
      isunder = false,
      isroundstarting = true,
      isroundfinished = false,
      isroundevaluated = false,
      isstable = false,
      iscooling = false,
      localgo = go.load();
    gat::pid::types::Real minimum, maximum, mean;
    while (localgo) {
      time = Clock::now();

      duration = time - starttime;
      dms = std::chrono::duration_cast<std::chrono::milliseconds>(duration);
      elapsed = static_cast<double>(dms.count()) / 1000.0;

      pidoutput << elapsed;

      switch (tuningmode) {
      case TuningMode::blackbox:
        pidoutput << separator << round;
        if (isroundstarting) {
          startroundelapsed = elapsed;
          isover = false;
          isunder = false;
          isstable = false;
          iscooling = false;
          isroundstarting = false;
          isroundfinished = false;
          isroundevaluated = false;
          minimum = std::numeric_limits<gatp::types::Real>::lowest();
          maximum = std::numeric_limits<gatp::types::Real>::max();
        }
        break;
      }

      nresult = gat::modbus::master::read::input(input);
      if (nresult == GOS_SUCCESS) {
        if (!baseline.has_value()) {
          baseline = input.Temperature +
            GOS_ARDUINO_TOOLS_MASTER_BASE_LINE_ADDITION;
        }
        pidoutput << separator << input.Status;
      } else {
        pidoutput << separator << -1;
        std::cerr << gatpm::master::report::error::last() << std::endl;
      }

      switch (tuningmode) {
      case TuningMode::blackbox:
        pidoutput << separator << variables.Kp;
        pidoutput << separator << variables.Ki;
        break;
      default:
        if (initialized.Kp.has_value()) {
          pidoutput << separator << holding.Kp;
        }
        if (initialized.Ki.has_value()) {
          pidoutput << separator << holding.Ki;
        }
        break;
      }
      if (initialized.Kd.has_value()) {
        pidoutput << separator << holding.Kd;
      }
      if (initialized.Manual.has_value()) {
        pidoutput << separator << holding.Manual;
      }
      if (initialized.Setpoint.has_value()) {
        pidoutput << separator << holding.Setpoint;
      }

      if (nresult == GOS_SUCCESS) {
        pidoutput
          << separator << input.Output
          << separator << input.Temperature;
        if (varmap.count(GOS_ARDT_MOD_INTERNAL)) {
          pidoutput << separator << input.Error
            << separator << input.Integral
            << separator << input.Derivative;
        }

        if (input.Temperature > maximum) {
          maximum = input.Temperature;
        }
        if (input.Temperature < minimum) {
          minimum = input.Temperature;
        }
        window.add(input.Temperature);
        mean = window.mean();

        switch (tuningmode) {
        case TuningMode::blackbox:
          assert(baseline.has_value());
          if (!isroundfinished) {
            if (iscooling) {
              isroundfinished = mean < baseline.get();
            } else {
              if (isstable) {
                statistics.add(input.Temperature);
                Duration curstableduration = time - stabletime;
                if (curstableduration > std::chrono::minutes(stableduration)) {
                  /* Stop the controller and force zero */
                  if (gat::modbus::master::write::force(GOT_PI_FORCE_IDLE)) {
                    iscooling = true;
                    coolingelapsed = elapsed;
                  }
                }
              } else {
                if (isunder) {
                  stabletime = time;
                  stableelapsed = elapsed;
                  isstable = true;
                } else {
                  if (isover) {
                    isunder = mean < setpoint;
                    if (isunder) {
                      underelapsed = elapsed;
                    }
                  } else {
                    isover = mean > setpoint;
                    if (isover) {
                      overelapsed = elapsed;
                    }
                  }
                }
              }
            }
          }
          break;
        }
      } else {
        pidoutput << separator << separator;
        if (varmap.count(GOS_ARDT_MOD_INTERNAL)) {
          pidoutput << separator << separator << separator;
        }
      }

      if (isroundfinished) {
        bool updateresult;

        if (!isroundevaluated) {
          size_t count;
          gat::pid::types::Real stablestderr, difference;
          gat::pid::types::Real differencesqrtsum = 0.0;
          for (const gat::pid::types::Real value : statistics.vector()) {
            difference = setpoint - value;
            differencesqrtsum += difference * difference;
          }
          stablestderr = ::sqrtf(differencesqrtsum /
            static_cast<gatp::types::Real>(statistics.count()));
          gat::pid::types::Real peakerror = maximum - setpoint;

          gat::modbus::master::Evaluation evaluation;
          evaluation.Kp = variables.Kp;
          evaluation.Ki = variables.Ki;
          evaluation.Kd = kd;
          evaluation.PeakError = peakerror;
          evaluation.StableError = stablestderr;
          gat::modbus::master::evaluate(evaluation);
          count = statistics.count();

          statistics.clear();
          window.clear();

          if (lastevaluation) {
            if (evaluation.Performance >= lastevaluation->Performance) {
              variables.Kp = lastevaluation->Kp;
              variables.Ki = lastevaluation->Ki;
            }
            gatp::tuning::black::box::compute::newtunings(parameters, variables);
          } else {
            gat::modbus::master::replace(lastevaluation, evaluation);
            gatp::tuning::black::box::compute::newtunings(parameters, variables);
          }

          (*tuningoutput) << startroundelapsed << separator << round
            << separator << variables.Kp
            << separator << variables.Ki
            << separator << kd
            << separator << maximum
            << separator << peakerror
            << separator << stablestderr
            << separator << evaluation.Performance;
          if (varmap.count(GOS_ARDT_MOD_INTERNAL)) {
            (*tuningoutput)
              << separator << overelapsed
              << separator << underelapsed
              << separator << stableelapsed
              << separator << coolingelapsed
              << separator << count;
          }
          (*tuningoutput) << std::endl;

          tuningoutput->flush();

          isroundevaluated = true;
        }

        updateresult =
          gatmm::write::tuning(variables.Kp, variables.Ki) ||
          gatmm::write::force(GOT_PI_FORCE_AUTO);
        if (updateresult) {
          round++;
          isroundstarting = true;
        }
      }

      pidoutput << std::endl;
      if (output) {
        output->flush();
      }

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
  } catch (::gos::arduino::tools::exception& er) {
    std::cerr << "Tool exception: "
      << er.what() << std::endl;
    return EXIT_FAILURE;
  } catch (po::error& er) {
    std::cerr << "Boost parsing program option exception: "
      << er.what() << std::endl;
    return EXIT_FAILURE;
  } catch (std::exception& ex) {
    std::cerr << "General parsing option exception: "
      << ex.what() << std::endl;
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "General parsing option exception" << std::endl;
    return EXIT_FAILURE;
  }

  goto gos_arduino_tools_pid_modbus_master_exit;

gos_arduino_tools_pid_modbus_master_exit_failure:
  retval = EXIT_FAILURE;

gos_arduino_tools_pid_modbus_master_exit:
  if (output) {
    output->flush();
    output->close();
  }
  switch (tuningmode) {
  case TuningMode::blackbox:
    if (tuningoutput) {
      tuningoutput->flush();
      tuningoutput->close();
    }
    gat::modbus::master::write::force(GOT_PI_FORCE_IDLE);
    break;
  default:
    if (final.has_value()) {
      gat::modbus::master::write::force(final.get());
    }
    break;
  }
  gatpm::master::disconnect();
  gatpm::master::shutdown();
  return retval;
}

namespace gos {
namespace arduino {
namespace tools {
namespace modbus {
namespace master {

gat::pid::types::Real evaluate(
  gat::pid::types::Real peakerror,
  gat::pid::types::Real stableerror) {
  return PeakFactor * peakerror + StableFactor * stableerror;
}

void evaluate(Evaluation& evaluation) {
  evaluation.Performance = evaluate(
    evaluation.PeakError,
    evaluation.StableError);
}

void replace(EvaluationPointer& last, const Evaluation& current) {
  if (!last) {
    last = std::make_unique<Evaluation>();
  }
  last->Kp = current.Kp;
  last->Ki = current.Ki;
  last->Kd = current.Kd;
  last->PeakError = current.PeakError;
  last->StableError = current.StableError;
  last->Performance = current.Performance;
}

namespace read {

int holding(gatpt::registry::Holding& holding, const int& maxretry) {
  gatpm::types::result result;
  for (int rt = 0; rt < maxretry; ++rt) {
    result = gatpm::master::read::holding(holding);
    if (result == gatpm::types::result::success) {
      break;
    } else {
      std::cerr << gatpm::master::report::error::last() << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
  }
  return result == gatpm::types::result::success ? EXIT_SUCCESS : EXIT_FAILURE;
}

int input(gatpt::registry::Input& input, const int& maxretry) {
  gatpm::types::result result;
  for (int rt = 0; rt < maxretry; ++rt) {
    result = gatpm::master::read::input(input);
    if (result == gatpm::types::result::success) {
      break;
    } else {
      std::cerr << gatpm::master::report::error::last() << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
  }
  return result == gatpm::types::result::success ? EXIT_SUCCESS : EXIT_FAILURE;
}

} // namespace read

namespace write {

bool tuning(
  const gatpt::Real& kp,
  const gatpt::Real& ki,
  const gatpt::Real& kd,
  const int& maxretry) {
  gatpmt::result result;
  for (int rt = 0; rt < maxretry; ++rt) {
    result = gatpm::master::write::tuning(kp, ki, kd);
    if (result == gatpm::types::result::success) {
      break;
    } else {
      std::cerr << gatpm::master::report::error::last() << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
  }
  return result == gatpm::types::result::success;
}

bool tuning(
  const gatpt::Real& kp,
  const gatpt::Real& ki,
  const int& maxretry) {
  gatpmt::result result;
  for (int rt = 0; rt < maxretry; ++rt) {
    result = gatpm::master::write::tuning(kp, ki);
    if (result == gatpm::types::result::success) {
      break;
    } else {
      std::cerr << gatpm::master::report::error::last() << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
  }
  return result == gatpm::types::result::success;
}


bool kp(const gatpt::Real& kp, const int& maxretry) {
  gatpmt::result result;
  for (int rt = 0; rt < maxretry; ++rt) {
    result = gatpm::master::write::kp(kp);
    if (result == gatpm::types::result::success) {
      break;
    } else {
      std::cerr << gatpm::master::report::error::last() << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
  }
  return result == gatpm::types::result::success;
}

bool ki(const gatpt::Real& ki, const int& maxretry) {
  gatpmt::result result;
  for (int rt = 0; rt < maxretry; ++rt) {
    result = gatpm::master::write::ki(ki);
    if (result == gatpm::types::result::success) {
      break;
    } else {
      std::cerr << gatpm::master::report::error::last() << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
  }
  return result == gatpm::types::result::success;
}

bool kd(const gatpt::Real& kd, const int& maxretry) {
  gatpmt::result result;
  for (int rt = 0; rt < maxretry; ++rt) {
    result = gatpm::master::write::kd(kd);
    if (result == gatpm::types::result::success) {
      break;
    } else {
      std::cerr << gatpm::master::report::error::last() << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
  }
  return result == gatpm::types::result::success;
}

bool setpoint(const gatpt::Real& setpoint, const int& maxretry) {
  gatpmt::result result;
  for (int rt = 0; rt < maxretry; ++rt) {
    result = gatpm::master::write::setpoint(setpoint);
    if (result == gatpm::types::result::success) {
      break;
    } else {
      std::cerr << gatpm::master::report::error::last() << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
  }
  return result == gatpm::types::result::success;
}

bool manual(const gatpt::Unsigned& manual, const int& maxretry) {
  gatpmt::result result;
  for (int rt = 0; rt < maxretry; ++rt) {
    result = gatpm::master::write::manual(manual);
    if (result == gatpm::types::result::success) {
      break;
    } else {
      std::cerr << gatpm::master::report::error::last() << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
  }
  return result == gatpm::types::result::success;
}

bool force(const gatpt::Unsigned& force, const int& maxretry) {
  gatpmt::result result;
  for (int rt = 0; rt < maxretry; ++rt) {
    result = gatpm::master::write::force(force);
    if (result == gatpm::types::result::success) {
      break;
    } else {
      std::cerr << gatpm::master::report::error::last() << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
  }
  return result == gatpm::types::result::success;
}

bool interval(const gatpt::Unsigned& interval, const int& maxretry) {
  gatpmt::result result;
  for (int rt = 0; rt < maxretry; ++rt) {
    result = gatpm::master::write::interval(interval);
    if (result == gatpm::types::result::success) {
      break;
    } else {
      std::cerr << gatpm::master::report::error::last() << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
  }
  return result == gatpm::types::result::success;
}

} // namespace write


namespace initial {
namespace apply {
int tuning(
  gat::modbus::master::Initialized& initialized,
  po::variables_map& varmap,
  gatp::tuning::black::box::Variables& variables,
  const gatp::tuning::black::box::Parameters& parameters,
  const TuningMode& mode) {

  if (varmap.count(GOS_ARDT_MOD_KP) > 0) {
    initialized.Kp = varmap[GOS_ARDT_MOD_KP].as<gatpt::Real>();
  }
  if (varmap.count(GOS_ARDT_MOD_KI) > 0) {
    initialized.Ki = varmap[GOS_ARDT_MOD_KI].as<gatpt::Real>();
  }
  if (varmap.count(GOS_ARDT_MOD_KD) > 0) {
    initialized.Kd = varmap[GOS_ARDT_MOD_KD].as<gatpt::Real>();
  }
  if (varmap.count(GOS_ARDT_MOD_MANUAL) > 0) {
    initialized.Manual =
      varmap[GOS_ARDT_MOD_MANUAL].as<gatp::types::Unsigned>();
  }
  if (varmap.count(GOS_ARDT_MOD_SETPOINT) > 0) {
    initialized.Setpoint =
      varmap[GOS_ARDT_MOD_SETPOINT].as<gatp::types::Real>();
  }

  switch (mode) {
  case TuningMode::blackbox:
    if (initialized.Kp.has_value() && initialized.Ki.has_value()) {
      gatp::tuning::black::box::initialize(
        parameters,
        variables,
        initialized.Kp.get(),
        initialized.Ki.get());
    } else {
      gatp::tuning::black::box::initialize(parameters, variables);
      initialized.Kp = variables.Kp;
      initialized.Ki = variables.Ki;
    }
    break;
  default:
    break;
  }

  if (
    initialized.Kp.has_value() &&
    initialized.Ki.has_value() &&
    initialized.Kd.has_value()) {
    if (!gat::modbus::master::write::tuning(
      initialized.Kp.get(),
      initialized.Ki.get(),
      initialized.Kd.get())) {
      return EXIT_FAILURE;
    }
  } else {
    if (
      initialized.Kp.has_value() &&
      initialized.Ki.has_value()) {
      if (!gat::modbus::master::write::tuning(
        initialized.Kp.get(),
        initialized.Ki.get())) {
        return EXIT_FAILURE;
      }
    } else {
      if (initialized.Kp.has_value()) {
        if (!gat::modbus::master::write::kp(initialized.Kp.get())) {
          return EXIT_FAILURE;
        }
      }
      if (initialized.Ki.has_value()) {
        if (!gat::modbus::master::write::ki(initialized.Ki.get())) {
          return EXIT_FAILURE;
        }
      }
    }
    if (initialized.Kd.has_value()) {
      if (!gat::modbus::master::write::kd(initialized.Kd.get())) {
        return EXIT_FAILURE;
      }
    }
  }

  if (initialized.Setpoint.has_value()) {
    if (!gat::modbus::master::write::setpoint(initialized.Setpoint.get())) {
      return EXIT_FAILURE;
    }
  } else if (initialized.Manual.has_value()) {
    if (!gat::modbus::master::write::manual(initialized.Manual.get())) {
      return EXIT_FAILURE;
    }
  }

  return EXIT_SUCCESS;
}
} // namespace apply
} // namespace initial

} // namespace master
} // namespace modbus
} // namespace tools
} // namespace arduino
} // namespace gos
