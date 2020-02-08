#include <iostream>

#include <gos/arduino/test/tools/modbus/arduinoslave.h>

namespace gos {
namespace arduino {
namespace test {
namespace tools {
namespace modbus {

typedef std::unique_ptr<Modbus> ModbusPointer;

static ModbusPointer _modbus;

namespace read {
uint8_t coils(uint8_t fc, uint16_t address, uint16_t length);
namespace discrete {
uint8_t inputs(uint8_t fc, uint16_t address, uint16_t length);
} // namespace discrete
namespace input {
uint8_t registers(uint8_t fc, uint16_t address, uint16_t length);
} // namespace input
namespace holding {
uint8_t registers(uint8_t fc, uint16_t address, uint16_t length);
} // namespace holding
} // namespace read
namespace write {
uint8_t coils(uint8_t fc, uint16_t address, uint16_t length);
namespace holding {
uint8_t registers(uint8_t fc, uint16_t address, uint16_t length);
} // namespace holding
} // namespace write


modbusslave::modbusslave(Stream& stream, const int& id) {
  _modbus = std::make_unique<Modbus>(stream, id);
}

void modbusslave::begin(const int& baud) {
  if (_modbus) {
    _modbus->cbVector[CB_READ_COILS] = read::coils;
    _modbus->cbVector[CB_READ_DISCRETE_INPUTS] = read::discrete::inputs;
    _modbus->cbVector[CB_WRITE_COILS] = write::coils;
    _modbus->cbVector[CB_READ_INPUT_REGISTERS] = read::input::registers;
    _modbus->cbVector[CB_READ_HOLDING_REGISTERS] = read::holding::registers;
    _modbus->cbVector[CB_WRITE_HOLDING_REGISTERS] = write::holding::registers;
    _modbus->begin(baud);
  }
}

void modbusslave::loop() {
  uint8_t pool = _modbus->poll();
}

namespace read {
uint8_t coils(uint8_t fc, uint16_t address, uint16_t length) {
  if (_modbus) {
    std::cout << "Arduino Modbus Function " << static_cast<int>(fc)
      << ": Reading " << length
      << " coils from address " << address << std::endl;
    for (uint16_t i = 0; i < length; ++i) {
      _modbus->writeCoilToBuffer(i, false);
    }
  }
  return STATUS_OK;
}
namespace discrete {
uint8_t inputs(uint8_t fc, uint16_t address, uint16_t length) {
  if (_modbus) {
    std::cout << "Arduino Modbus Function " << static_cast<int>(fc)
      << ": Reading " << length
      << " discrete inputs from address " << address << std::endl;
    for (uint16_t i = 0; i < length; ++i) {
      _modbus->writeDiscreteInputToBuffer(i, false);
    }
  }
  return STATUS_OK;
}
} // namespace discrete
namespace input {
uint8_t registers(uint8_t fc, uint16_t address, uint16_t length) {
  std::cout << "Arduino Modbus Function " << static_cast<int>(fc)
    << ": Reading " << length
    << " input registers from address " << address << std::endl;
  return STATUS_OK;
}
} // namespace input
namespace holding {
uint8_t registers(uint8_t fc, uint16_t address, uint16_t length) {
  std::cout << "Arduino Modbus Function " << static_cast<int>(fc)
    << ": Reading " << length
    << " holding registers from address " << address << std::endl;
  return STATUS_OK;
}
} // namespace holding
} // namespace read
namespace write {
uint8_t coils(uint8_t fc, uint16_t address, uint16_t length) {
  std::cout << "Arduino Modbus Function " << static_cast<int>(fc)
    << ": Writing " << length
    << " coils from address " << address << std::endl;
  return STATUS_OK;
}
namespace holding {
uint8_t registers(uint8_t fc, uint16_t address, uint16_t length) {
  std::cout << "Arduino Modbus Function " << static_cast<int>(fc)
    << ": Writing " << length
    << " holding registers from address " << address << std::endl;
  return STATUS_OK;
}
} // namespace holding
} // namespace write

} // namespace modbus
} // namespace tools
} // namespace test
} // namespace arduino 
} // namespace gos
