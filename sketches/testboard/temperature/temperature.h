#ifndef GOS_SKETCHES_TEST_BOARD_TEMPERATURE_H_
#define GOS_SKETCHES_TEST_BOARD_TEMPERATURE_H_

#include <arduinotick.h>
#include <gatlstatistics.h>
#include <gatlbuffer.h>

#define TEXT_ID_1 "1:"
#define TEXT_ID_2 "2:"

#define INTERVAL_MAX_31865        500
#define INTERVAL_MAX_6675        2000

struct Text {
  Text() : Text(nullptr), Length() {}
  const char* Text;
  uint8_t Length;
};

struct Display {
  Display(const char* id, const uint8_t& size) : Id(id, size) {}
  ::gos::atl::buffer::Holder<> Buffer;
  ::gos::atl::buffer::Holder<> Id;
};

struct Sensor {
  Sensor(
    const unsigned long& interval,
    const char* id,
    const uint8_t& size) :
    Value(),
    Timer(interval),
    Display(id, size) {}
  double Value;
  Tick Timer;
  struct Text Error;
  struct Display Display;
#ifndef NO_STATISTICS
  ::gos::atl::statistics::Set<double> Set;
#endif
};
typedef struct Sensor Sensor;

struct Sensor sensormax31865(
  INTERVAL_MAX_31865,
  TEXT_ID_1,
  sizeof(TEXT_ID_1));

struct Sensor sensormax6675(
  INTERVAL_MAX_6675,
  TEXT_ID_2,
  sizeof(TEXT_ID_2));

#endif
