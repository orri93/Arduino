#ifndef GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_EEPROM_H_
#define GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_EEPROM_H_

#define GOS_TC_EEPROM_INDEX_COILS      0x0000
#define GOS_TC_EEPROM_INDEX_INTERVAL   0x0001
#define GOS_TC_EEPROM_INDEX_MANUAL     0x0003
#define GOS_TC_EEPROM_INDEX_SETPOINT   0x0005
#define GOS_TC_EEPROM_INDEX_KP         0x0009
#define GOS_TC_EEPROM_INDEX_KITI       0x000d
#define GOS_TC_EEPROM_INDEX_KDTD       0x0011
#define GOS_TC_EEPROM_INDEX_MIN_SENS   0x0013
#define GOS_TC_EEPROM_INDEX_MAX_SENS   0x0015

namespace gos {
namespace temperature {
namespace eeprom {

namespace retrieve {
void initial();
}

} // namespace eeprom
} // namespace temperature
} // namespace gos

#endif
