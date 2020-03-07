#include <Arduino.h>

#include <ModbusSlave.h>

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

uint8_t ReadCoils(uint8_t function, uint16_t start, uint16_t length);
//uint8_t ReadDiscreteInputs(uint8_t function, uint16_t start, uint16_t length);
//uint8_t ReadHoldingRegisters(uint8_t function, uint16_t start, uint16_t length);
//uint8_t ReadInputRegisters(uint8_t function, uint16_t start, uint16_t length);
uint8_t WriteCoils(uint8_t function, uint16_t start, uint16_t length);
//uint8_t WriteHoldingRegisters(uint8_t function, uint16_t start, uint16_t length);

uint8_t coils = 0;

Modbus slave(MODBUS_SLAVE_ID, PIN_RS485_MODBUS_TE);

void setup() {
  pinMode(PIN_LED_RED_A, OUTPUT);
  pinMode(PIN_LED_RED_B, OUTPUT);
  pinMode(PIN_LED_BLUE_A, OUTPUT);
  pinMode(PIN_LED_BLUE_B, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_YELLOW, OUTPUT);
  slave.cbVector[CB_READ_COILS] = ReadCoils;
  //slave.cbVector[CB_READ_DISCRETE_INPUTS] = ReadDiscreteInputs;
  //slave.cbVector[CB_READ_HOLDING_REGISTERS] = ReadHoldingRegisters;
  //slave.cbVector[CB_READ_INPUT_REGISTERS] = ReadInputRegisters;
  slave.cbVector[CB_WRITE_COILS] = WriteCoils;
  //slave.cbVector[CB_WRITE_HOLDING_REGISTERS] = WriteHoldingRegisters;
  Serial.begin(MODBUS_BAUD);
  slave.begin(MODBUS_BAUD);
}

void loop() {
  slave.poll();
}

uint8_t ReadCoils(uint8_t function, uint16_t start, uint16_t length) {
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
#ifdef DO_SOMETHING
  for (uint16_t i = 0; i < length; ++i) {
    if (start + i < 8) {
      slave.writeCoilToBuffer(i, bitRead(coils, start + i));
    } else {
      return STATUS_ILLEGAL_DATA_ADDRESS;
    }
  }
#endif
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return STATUS_OK;
}

uint8_t WriteCoils(uint8_t function, uint16_t start, uint16_t length) {
  digitalWrite(PIN_LED_MODBUS_WRITE, HIGH);
#ifdef DO_SOMETHING
  for (uint16_t i = 0; i < length; ++i) {
    if (start + i < 8) {
      if (slave.readCoilFromBuffer(i)) {
        bitSet(coils, start + i);
      } else {
        bitClear(coils, start + i);
      }
    } else {
      return STATUS_ILLEGAL_DATA_ADDRESS;
    }
  }
#endif
  digitalWrite(PIN_LED_MODBUS_WRITE, LOW);
  return STATUS_OK;
}
