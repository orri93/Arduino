#include "pid.h"
#include "variable.h"

namespace gt = ::gos::temperature;
namespace gtv = ::gos::temperature::variables;

namespace gatl = ::gos::atl;
namespace gatlp = ::gos::atl::pid;
namespace gatlpt = ::gos::atl::pid::time;

namespace gos {
namespace temperature {
namespace pid {

Parameter parameter;
Variable variable;

namespace tune {
Tune k;
TimeTune t;
void calculate() {
  if (bitRead(gtv::modbus::coils, GOS_TCV_COIL_BIT_TUNE_TIME_MASTER)) {
    switch (lowByte(gtv::pid::tune::time::unit)) {
    case GOT_PI_TUNE_TIME_UNIT_MILLISECONDS:
      k.Ki = gatlpt::milliseconds::Ki(gt::pid::parameter.Kp, t.Ti);
      k.Kd = gatlpt::milliseconds::Kd(gt::pid::parameter.Kp, t.Td);
      break;
    case GOT_PI_TUNE_TIME_UNIT_SECONDS:
      k.Ki = gatlpt::seconds::Ki(gt::pid::parameter.Kp, t.Ti);
      k.Kd = gatlpt::seconds::Kd(gt::pid::parameter.Kp, t.Td);
      break;
    case GOT_PI_TUNE_TIME_UNIT_DEFAULT:
    case GOT_PI_TUNE_TIME_UNIT_MINUTES:
      k.Ki = gatlpt::minutes::Ki(gt::pid::parameter.Kp, t.Ti);
      k.Kd = gatlpt::minutes::Kd(gt::pid::parameter.Kp, t.Td);
      break;
    }
  } else {
    switch (gtv::pid::tune::time::unit) {
    case GOT_PI_TUNE_TIME_UNIT_MILLISECONDS:
      t.Ti = gatlpt::milliseconds::Ti(gt::pid::parameter.Kp, k.Ki);
      t.Td = gatlpt::milliseconds::Td(gt::pid::parameter.Kp, k.Kd);
      break;
    case GOT_PI_TUNE_TIME_UNIT_SECONDS:
      t.Ti = gatlpt::seconds::Ti(gt::pid::parameter.Kp, k.Ki);
      t.Td = gatlpt::seconds::Td(gt::pid::parameter.Kp, k.Kd);
      break;
    case GOT_PI_TUNE_TIME_UNIT_DEFAULT:
    case GOT_PI_TUNE_TIME_UNIT_MINUTES:
      t.Ti = gatlpt::minutes::Ti(gt::pid::parameter.Kp, k.Ki);
      t.Td = gatlpt::minutes::Td(gt::pid::parameter.Kp, k.Kd);
      break;
    }

  }
}
void tunings() {
  if (bitRead(gtv::modbus::coils, GOS_TCV_COIL_BIT_TUNE_TIME_MASTER)) {
    switch (highByte(gtv::pid::tune::time::unit)) {
    case GOT_PI_TUNE_TIME_UNIT_MILLISECONDS:
      gatlpt::milliseconds::tunings(gt::pid::variable, gt::pid::parameter, t);
      break;
    case GOT_PI_TUNE_TIME_UNIT_SECONDS:
      gatlpt::seconds::tunings(gt::pid::variable, gt::pid::parameter, t);
      break;
    case GOT_PI_TUNE_TIME_UNIT_DEFAULT:
    case GOT_PI_TUNE_TIME_UNIT_MINUTES:
      gatlpt::minutes::tunings(gt::pid::variable, gt::pid::parameter, t);
      break;
    }
  } else {
    switch (highByte(gtv::pid::tune::time::unit)) {
    case GOT_PI_TUNE_TIME_UNIT_MILLISECONDS:
      gatlpt::milliseconds::tunings(gt::pid::variable, gt::pid::parameter, k);
      break;
    case GOT_PI_TUNE_TIME_UNIT_SECONDS:
      gatlpt::seconds::tunings(gt::pid::variable, gt::pid::parameter, k);
      break;
    case GOT_PI_TUNE_TIME_UNIT_DEFAULT:
    case GOT_PI_TUNE_TIME_UNIT_MINUTES:
      gatlpt::minutes::tunings(gt::pid::variable, gt::pid::parameter, k);
      break;
    }
  }
}
} // namespace tune

} // namespace pid
} // namespace temperature
} // namespace gos
