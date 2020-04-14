#ifndef GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_MACRO_H_
#define GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_TEMPERATUREPID_MACRO_H_

#define PIN_HEATER                           6
#define PIN_MAX31865_CS                      7
#define PIN_MAX6675_CS                       8
#define PIN_DS18B20                          9

#define PIN_RS485_MODBUS_RX                  0
#define PIN_RS485_MODBUS_TX                  1
#define PIN_RS485_MODBUS_TE                  2

#define PIN_LED_RED                          6
#define PIN_LED_BLUE                         5

#define PIN_LED_GREEN                        3
#define PIN_LED_YELLOW                       4

#define PIN_LED_MODBUS_READ      PIN_LED_GREEN
#define PIN_LED_MODBUS_WRITE    PIN_LED_YELLOW

#define PIN_BUTTON                          A0
#define PIN_POTENTIOMETER                   A2

/* For PT100 set type to 1 and for PT1000 set type to 2 */
#define MAX31865_RTD_TYPE      RTD_TYPE_PT1000

// set to 2WIRE or 4WIRE as necessary
#define MAX31865_WIRES              RTD_3_WIRE

#define MODBUS_BAUD                       9600

#define MODBUS_SLAVE_ID                      1

#define MODBUS_BUFFER_SIZE                  64

#ifdef USE_FLOAT_FOR_REAL
#define SETPOINT_MININUM                  0.0F
#define SETPOINT_MAXIMUM                300.0F
#else
#define SETPOINT_MININUM                   0.0
#define SETPOINT_MAXIMUM                 300.0
#endif

#define PID_MINIMUM_OUTPUT                   0
#define PID_MAXIMUM_OUTPUT                 255

#define ANALOG_MININUM                       0
#define ANALOG_MAXIMUM                    1024

#define DEFAULT_INTERVAL                  2000

#define DISPLAY_INTERVAL                   250

#endif
