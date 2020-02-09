#ifndef GOS_ARDUINO_SKETCHES_TESTBOARD_SIMPLE_MODBUS_H_
#define GOS_ARDUINO_SKETCHES_TESTBOARD_SIMPLE_MODBUS_H_

#include <gatlmodbus.h>

#define GOS_MODBUS_DO_NOTHING

#define PIN_RS485_MODBUS_RX                  0
#define PIN_RS485_MODBUS_TX                  1
#define PIN_RS485_MODBUS_TE                  2

#define PIN_LED_RED_A                        3
#define PIN_LED_RED_B                       11

#define PIN_LED_BLUE_A                       6
#define PIN_LED_BLUE_B                       5

#define PIN_LED_GREEN                        9
#define PIN_LED_YELLOW                      10

#define PIN_LED_MODBUS_READ      PIN_LED_GREEN
#define PIN_LED_MODBUS_WRITE    PIN_LED_YELLOW

#define PIN_BUTTON_A                         8
#define PIN_BUTTON_B                         7

#define PIN_POTENTIOMETER                   A0

#define MODBUS_BAUD                       9600

#define MODBUS_SLAVE_ID                      1

#define MODBUS_BUFFER_SIZE                  64


namespace gos {
namespace modbus {

namespace base {
typedef ::gos::atl::modbus::Handler<> Handler;
}

class Handler : public virtual base::Handler {
public:
  MODBUS_TYPE_RESULT ReadCoils(
    const MODBUS_TYPE_FUNCTION& function,
    const MODBUS_TYPE_DEFAULT& address,
    const MODBUS_TYPE_DEFAULT& length);
  MODBUS_TYPE_RESULT ReadDiscretes(
    const MODBUS_TYPE_FUNCTION& function,
    const MODBUS_TYPE_DEFAULT& address,
    const MODBUS_TYPE_DEFAULT& length);
  MODBUS_TYPE_RESULT ReadHoldingRegisters(
    const MODBUS_TYPE_FUNCTION& function,
    const MODBUS_TYPE_DEFAULT& address,
    const MODBUS_TYPE_DEFAULT& length);
  MODBUS_TYPE_RESULT ReadInputRegisters(
    const MODBUS_TYPE_FUNCTION& function,
    const MODBUS_TYPE_DEFAULT& address,
    const MODBUS_TYPE_DEFAULT& length);
  MODBUS_TYPE_RESULT WriteCoils(
    const MODBUS_TYPE_FUNCTION& function,
    const MODBUS_TYPE_DEFAULT& address,
    const MODBUS_TYPE_DEFAULT& length);
  MODBUS_TYPE_RESULT WriteHoldingRegisters(
    const MODBUS_TYPE_FUNCTION& function,
    const MODBUS_TYPE_DEFAULT& address,
    const MODBUS_TYPE_DEFAULT& length);
  MODBUS_TYPE_RESULT ReadExceptionStatus(const MODBUS_TYPE_FUNCTION& function);
};

void initialize();

namespace binding {
void create();
} /* End of modbus binding name-space */
namespace buffer {
typedef ::gos::atl::buffer::Holder<uint16_t, MODBUS_TYPE_BUFFER> Holder;
Holder request(MODBUS_BUFFER_SIZE);
Holder response(MODBUS_BUFFER_SIZE);
}

extern Handler handler;

typedef ::gos::atl::modbus::structures::Parameter<> Parameter;
typedef ::gos::atl::modbus::structures::Variable<> Variable;

Parameter parameter;
Variable variable;

} // namespace modbus
} // namespace gos

#endif
