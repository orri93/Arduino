#ifndef GOS_ARDUINO_SKETCHES_TESTBOARD_SIMPLE_MODBUS_MACRO_H_
#define GOS_ARDUINO_SKETCHES_TESTBOARD_SIMPLE_MODBUS_MACRO_H_

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

#define GOS_BARRAY_BINDING

#endif
