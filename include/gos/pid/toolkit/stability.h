#ifndef GOS_PID_TOOLKIT_STABILITY_H_
#define GOS_PID_TOOLKIT_STABILITY_H_

#include <boost/accumulators/accumulators.hpp>

namespace gos {
namespace pid {
namespace stability {


template<typename T>
struct Parameter {
  ::gos::atl::type::range<O> Range;
  I Setpoint;
  P Time;
  P Kp;
  bool PonE;
};


void evaluate() {

}

}
}
}

#endif
