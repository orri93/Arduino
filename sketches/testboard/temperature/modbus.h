#ifndef GOS_SKETCHES_TEST_BOARD_TEMPERATURE_MODBUS_H_
#define GOS_SKETCHES_TEST_BOARD_TEMPERATURE_MODBUS_H_

#include <ModbusSlave.h>

#define MODBUS_SLAVE_ID     11
#define MODBUS_BAUD       9600

namespace gos {
namespace modbus {

extern Modbus slave;

void begin();

/* 0x01 Read Coils */
//uint8_t read_coils(
//  uint8_t fc,
//  uint16_t startaddress,
//  uint16_t length);

/* 0x02 Read Discrete Inputs */
uint8_t read_discrete_inputs(
  uint8_t fc,
  uint16_t startaddress,
  uint16_t length);

/* 0x03 Read Multiple Holding Registers */
uint8_t read_holding_registers(
  uint8_t fc,
  uint16_t startaddress,
  uint16_t length);

/* 0x04 Read Input Registers */
uint8_t read_input_registers(
  uint8_t fc,
  uint16_t startaddress,
  uint16_t length);

/* 0x05 Write Single Coil and 0x0f Write Multiple Coils */
//uint8_t write_coils(
//  uint8_t fc,
//  uint16_t startaddress,
//  uint16_t length);

/* 0x06 Write Single Holding Register, 0x10 Write Multiple Holding Registers */
uint8_t write_holding_registers(
  uint8_t fc,
  uint16_t startaddress,
  uint16_t length);

}
}

#endif

