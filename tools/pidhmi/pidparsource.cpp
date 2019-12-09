#include <pidparsource.h>

namespace gos {
namespace arduino {
namespace tools {
namespace pid {
namespace datasource {

Parameters::Parameters(QObject* parent) : QObject(parent) {
}

/* PID Parameters */
const uint16_t& Parameters::manual() const {
  return manual_;
}
const float& Parameters::setpoint() const {
  return setpoint_;
}

const float& Parameters::kp() const { return kp_; }
const float& Parameters::ki() const { return ki_; }
const float& Parameters::kd() const { return kd_; }
const float& Parameters::ti() const { return ti_; }
const float& Parameters::td() const { return td_; }

void Parameters::setManual(const uint16_t& value) {
  if (manual_ != value) {
    manual_ = value;
    emit manualChanged();
  }
}
void Parameters::setSetpoint(const float& value) {
  if (setpoint_ != value) {
    setpoint_ = value;
    emit setpointChanged();
  }
}
void Parameters::setKp(const float& value) {
  if (kp_ != value) {
    kp_ = value;
    emit kpChanged();
  }
}
void Parameters::setKi(const float& value) { }
void Parameters::setKd(const float& value) { }
void Parameters::setTi(const float& value) { }
void Parameters::setTd(const float& value) { }

}
}
}
}
}
