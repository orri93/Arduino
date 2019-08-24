#include "modbus.h"

namespace gos {
namespace modbus {

Modbus slave;

void begin() {
  // RS485 control pin must be output
  pinMode(PIN_RS485_MODBUS_TE, OUTPUT);

  //slave.cbVector[CB_READ_COILS] = read_coils;
  slave.cbVector[CB_READ_DISCRETE_INPUTS] = read_discrete_inputs;
  slave.cbVector[CB_READ_INPUT_REGISTERS] = read_input_registers;
  slave.cbVector[CB_READ_HOLDING_REGISTERS] = read_holding_registers;
  //slave.cbVector[CB_WRITE_COILS] = write_coils;
  slave.cbVector[CB_WRITE_HOLDING_REGISTERS] = write_holding_registers;

  Serial.begin(MODBUS_BAUD);
  slave.begin(MODBUS_BAUD);
}

}
}

#endif

