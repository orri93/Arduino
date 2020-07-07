#include <random>
#include <memory>

#include <gos/arduino/tools/utilities.h>

#include <gos/arduino/tools/pid/tuning/blackbox.h>

namespace gat = ::gos::arduino::tools;
namespace gatu = ::gos::arduino::tools::utilities;
namespace gatp = ::gos::arduino::tools::pid;
namespace gatpt = ::gos::arduino::tools::pid::types;

namespace gos {
namespace arduino {
namespace tools {
namespace pid {
namespace tuning {
namespace black {
namespace box {

typedef ::std::default_random_engine Engine;
typedef ::std::unique_ptr<Engine> EnginePointer;
typedef ::std::normal_distribution<gatpt::Real> Distribution;
typedef ::std::unique_ptr<Distribution> DistributionPointer;

static EnginePointer _engine;
static DistributionPointer _distribution;

static const gatpt::Real Mean = 0.0;

void initialize(
  const Parameters& parameters,
  Variables& variables,
  const gatpt::Real& kp,
  const gatpt::Real& ki) {
  variables.Kp = kp;
  variables.Ki = ki;
  _engine = ::std::make_unique<Engine>();
  _distribution = ::std::make_unique<Distribution>(Mean, parameters.Sd);
}

void initialize(const Parameters& parameters, Variables& variables) {
  initialize(
    parameters,
    variables,
    gatu::midpoint(parameters.Kp),
    gatu::midpoint(parameters.Ki));
}

void seed(const gatpt::Real& seed) {
  if (_engine) {
    _engine->seed(seed);
  }
}

namespace random {
gatpt::Real next() {
  return (*_distribution)(*_engine);
}
}

namespace compute {
void newtunings(const Parameters& parameters, Variables& variables) {
  gatpt::Real newkp = variables.Kp + random::next() * gatu::scale(parameters.Kp);
  gatpt::Real newki = variables.Ki + random::next() * gatu::scale(parameters.Ki);
  variables.Kp = gat::utilities::restrict<gatpt::Real>(parameters.Kp, newkp);
  variables.Ki = gat::utilities::restrict<gatpt::Real>(parameters.Ki, newki);
}
}

} // namespace box
} // namespace black
} // namespace tuning
} // namespace pid
} // namespace tools
} // namespace arduino 
} // namespace gos

