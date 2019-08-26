#ifndef GOS_SKETCHES_TEST_BOARD_TEMPERATURE_H_
#define GOS_SKETCHES_TEST_BOARD_TEMPERATURE_H_

#include <arduinotick.h>
#include <gatlstatistics.h>
#include <gatlbuffer.h>

#define TEXT_ID_1 "1:"
#define TEXT_ID_2 "2:"

#define INTERVAL_MAX_31865        500
#define INTERVAL_MAX_6675        2000

enum Coil {
  CoilManualAutomatic = 0x0000,
  CoilPonMPonE,
  CoilResetSum
};

enum DiscreteInput {
  DiscreteInputUnderOverTarget = 0x0000
};

enum HoldingRegistry {
  HoldingRegistrySensorSelect = 0x0000,
  HoldingRegistryManual,
  HoldingRegistryMinimum,
  HoldingRegistryMaximum,
  HoldingRegistrySampletime,
  HoldingRegistrySetpointWord1,
  HoldingRegistrySetpointWord2,
  HoldingRegistryKpWord1,
  HoldingRegistryKpWord2,
  HoldingRegistryKiWord1,
  HoldingRegistryKiWord2,
  HoldingRegistryKdWord1,
  HoldingRegistryKdWord2
};

enum InputRegistry {
  InputRegistryStatus = 0x0000,
  InputRegistrySsrValue,
  InputRegistryInputWord1,
  InputRegistryInputWord2,
  InputRegistryMainSensorStatus,
  InputRegistryMainSensorWord1,
  InputRegistryMainSensorWord2,
  InputRegistryMainSensorMedianWord1,
  InputRegistryMainSensorMedianWord2,
  InputRegistrySecondSensorStatus,
  InputRegistrySecondSensorWord1,
  InputRegistrySecondSensorWord2,
  InputRegistrySecondSensorMedianWord1,
  InputRegistrySecondSensorMedianWord2
};

namespace gos {
namespace temperature {

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

struct ModbusAddress {
  uint16_t Status;
  uint16_t ValueL;
  uint16_t ValueH;
  uint16_t MedianL;
  uint16_t MedianH;
};

struct Sensor {
  Sensor(const unsigned long& interval) : Value(), Timer(interval) {}
  double Value;
  Tick Timer;
  struct Text Error;
#ifndef NO_STATISTICS
  ::gos::atl::statistics::Set<double> Set;
#endif
};
typedef struct Sensor Sensor;

extern struct Sensor sensormax31865; (
  INTERVAL_MAX_31865,
  TEXT_ID_1,
  sizeof(TEXT_ID_1));

extern struct Sensor sensormax6675; (
  INTERVAL_MAX_6675,
  TEXT_ID_2,
  sizeof(TEXT_ID_2));

void begin();

void setsensorno(const uint8_t& no);
uint8_t getsensorno();

Sensor* getsensor(const uint8_t& index);
Sensor* getmain();
Sensor* getsecond();

}
}

#endif
