#ifndef MASTER_H
#define MASTER_H

#include <string>

#include <pidparsource.h>

namespace gos {
namespace arduino {
namespace tools {
namespace pid {
namespace master {

namespace report {
namespace error {
errno_t errorno();
std::string last();
}
}

enum class result {
  undefiend,
  success,
  failure,
  fatal,
  uninitialized,
  disconnected
};

result initialize(const char* device, const int& baud, const int& slaveid);

result connect();

result disconnect();

result shutdown();

namespace read {
result tunecoil(bool& output);
result inputs(uint16_t& output, float& temperature);
result holding(
  uint16_t& manual,
  float& setpoint,
  float& kp,
  float& ki,
  float& kd,
  float& ti,
  float& td);
}

namespace set {
result tunecoil(const bool& input);
result manual(const uint16_t& manual);
result setpoint(const float& setpoint);
result kp(const float& kp);
result ki(const float& ki);
result kd(const float& kd);
result ti(const float& ti);
result td(const float& td);
}

}
}
}
}
}

#endif
