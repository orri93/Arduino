#include "temperature.h"

namespace gos {
namespace temperature {

struct Sensor sensormax31865 (
  INTERVAL_MAX_31865,
  TEXT_ID_1,
  sizeof(TEXT_ID_1));

struct Sensor sensormax6675 (
  INTERVAL_MAX_6675,
  TEXT_ID_2,
  sizeof(TEXT_ID_2));

uint8_t sensor_select_no_ = 1;

void begin() {

}

}
}
