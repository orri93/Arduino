/*
 * ********************************************************
 * Mudbus Slave V0.0.1
 * ********************************************************
 *
 * Slave id 1
 *
 * MAX485 Wiring   Max485    Arduino UNO
 *                 1 RO      0 RX
 *                 2 RE      2 DI
 *                 3 DE      2 DI
 *                 4 DI      1 TX
 *                 5 GND       GND
 *                 6 A
 *                 7 B
 *                 8 VCC       5V
 * 
 * Display Wiring  Display   Arduino UNO
 *                 1 SDA     A4 (SDA)
 *                 2 SCL     A5 (SCL)
 *                 3 VCC     5V
 *                 4 GND     GND
 * 
 * ********************************************************/

#include <ModbusSlave.h>

#define PIN_RS485_MODBUS_TE      2

#define MODBUS_SLAVE_ID          1
#define RS485_BAUD            9600

#define PIN_COUNT          13 -  2
#define INPUT_COUNT        A5 - A0

#define INITIALIZE_PIN_PAUSE    10

namespace gos {
namespace modbus {

static uint16_t address, i;

static uint8_t result;

namespace pin {
const uint8_t DigitalOutput = 0b00;
const uint8_t DigitalInput  = 0b01;
const uint8_t AnalogOutput  = 0b10;
const uint8_t DigitalInputUp = 0b101;

uint8_t configuration[PIN_COUNT] = {
  DigitalOutput,     /* D03~ */
  DigitalOutput,   /* D04  */
  DigitalOutput,     /* D05~ */
  DigitalOutput,     /* D06~ */
  DigitalOutput,   /* D07  */
  DigitalOutput,   /* D08  */
  DigitalOutput,     /* D09~ */
  DigitalOutput,     /* D10~ */
  DigitalOutput,     /* D11~ */
  DigitalOutput,   /* D12  */
  DigitalOutput    /* D13  */
};
void configure();
namespace is {
bool input(const uint8_t& index);
namespace digital {
bool output(const uint8_t& index);
}
namespace analog {
bool output(const uint8_t& index);
}
}
}

uint8_t coils[2];

uint8_t holding[PIN_COUNT];

uint16_t value;

Modbus slave(MODBUS_SLAVE_ID, PIN_RS485_MODBUS_TE);

/* 0x01 Read Coils */
uint8_t read_coils(uint8_t fc, uint16_t startaddress, uint16_t length) {
  result = STATUS_OK;
  for (i = 0; i < length; i++) {
    address = startaddress + i;
    if (address > 0 && address < 16) {
      if (address > 8) {
        slave.writeCoilToBuffer(i, bitRead(coils[1], address - 8));
      } else {
        slave.writeCoilToBuffer(i, bitRead(coils[0], address));
      }
    } else {
      result = STATUS_ILLEGAL_DATA_ADDRESS;
    }
  }
  return result;
}

/* 0x02 Read Discrete Inputs */
uint8_t read_discrete_inputs(uint8_t fc, uint16_t startaddress, uint16_t length) {
  result = STATUS_OK;
  for (i = 0; i < length; i++) {
    address = startaddress + i;
    if (address > 0 && address < PIN_COUNT) {
      if (pin::is::input(address)) {
        slave.writeDiscreteInputToBuffer(i, digitalRead(address + 2));
      } else {
        result = STATUS_ILLEGAL_DATA_VALUE;
      }
    } else {
      result = STATUS_ILLEGAL_DATA_ADDRESS;
    }
  }
  return result;
}

/* 0x03 Read Multiple Holding Registers */
uint8_t read_holding_registers(uint8_t fc, uint16_t startaddress, uint16_t length) {
  result = STATUS_OK;
  for (i = 0; i < length; i++) {
    address = startaddress + i;
    if (address > 0 && address < PIN_COUNT) {
      value = holding[address];
      slave.writeRegisterToBuffer(i, value);
    } else {
      result = STATUS_ILLEGAL_DATA_ADDRESS;
    }
  }
  return result;
}

/* 0x04 Read Input Registers */
uint8_t read_input_registers(uint8_t fc, uint16_t startaddress, uint16_t length) {
  result = STATUS_OK;
  for (i = 0; i < length; i++) {
    address = startaddress + i;
    if (address > 0 && address < A5 - A0) {
      slave.writeRegisterToBuffer(i, analogRead(A0 + address));
    } else {
      result = STATUS_ILLEGAL_DATA_ADDRESS;
    }
  }
  return result;
}

/* 0x05 Write Single Coil and 0x0f Write Multiple Coils */
uint8_t write_coils(uint8_t fc, uint16_t startaddress, uint16_t length) {
  result = STATUS_OK;
  for (i = 0; i < length; i++) {
    address = startaddress + i;
    if (address > 0 && address < 16) {
      if (address < PIN_COUNT && pin::is::digital::output(address)) {
        digitalWrite(address + 2, slave.readCoilFromBuffer(i) ? HIGH : LOW);
      }
      if (address > 8) {
        bitWrite(coils[1], address - 8, slave.readCoilFromBuffer(i));
      } else {
        bitWrite(coils[0], address, slave.readCoilFromBuffer(i));
      }
    } else {
      result = STATUS_ILLEGAL_DATA_ADDRESS;
    }
  }
  return result;
}

/* 0x06 Write Single Holding Register and 0x10 Write Multiple Holding Registers */
uint8_t write_holding_registers(uint8_t fc, uint16_t startaddress, uint16_t length) {
  result = STATUS_OK;
  for (i = 0; i < length; i++) {
    address = startaddress + i;
    if (address > 0 && address < PIN_COUNT) {
      value = slave.readRegisterFromBuffer(i);
      if (value <= 255) {
        holding[address] = static_cast<uint8_t>(value);
        if (pin::is::analog::output(address)) {
          analogWrite(address, value);
        }
      } else {
        result = STATUS_ILLEGAL_DATA_VALUE;
      }
    } else {
      result = STATUS_ILLEGAL_DATA_ADDRESS;
    }
  }
  return result;
}

}
}

void setup() {
  // RS485 control pin must be output
  pinMode(PIN_RS485_MODBUS_TE, OUTPUT);

  gos::modbus::pin::configure();

  gos::modbus::slave.cbVector[CB_READ_COILS] =
    gos::modbus::read_coils;
  gos::modbus::slave.cbVector[CB_READ_DISCRETE_INPUTS] =
    gos::modbus::read_discrete_inputs;
  gos::modbus::slave.cbVector[CB_READ_INPUT_REGISTERS] =
    gos::modbus::read_input_registers;
  gos::modbus::slave.cbVector[CB_READ_HOLDING_REGISTERS] =
    gos::modbus::read_holding_registers;
  gos::modbus::slave.cbVector[CB_WRITE_COILS] =
    gos::modbus::write_coils;
  gos::modbus::slave.cbVector[CB_WRITE_HOLDING_REGISTERS] =
    gos::modbus::write_holding_registers;

  Serial.begin(RS485_BAUD);
  gos::modbus::slave.begin(RS485_BAUD);
}

void loop() {
  gos::modbus::slave.poll();
}


namespace gos {
namespace modbus {

namespace pin {
void configure() {
  for (i = 2; i <= 13; i++) {
    switch (configuration[i - 2]) {
    case DigitalOutput:
      pinMode(i, OUTPUT);
#ifdef INITIALIZE_PIN_PAUSE
      digitalWrite(i, HIGH);
      delay(INITIALIZE_PIN_PAUSE);
#endif
      digitalWrite(i, LOW);
      break;
    case DigitalInput:
      pinMode(i, INPUT);
      break;
    case AnalogOutput:
      pinMode(i, OUTPUT);
#ifdef INITIALIZE_PIN_PAUSE
      for (uint8_t d = 0; d <= 255; ++d) {
        analogWrite(i, d);
      }
      for (uint8_t d = 255; d > 0; --d) {
        analogWrite(i, d);
      }
#endif
      analogWrite(i, 0);
      break;
    case DigitalInputUp:
      pinMode(i, INPUT_PULLUP);
      break;
    }
  }
}
namespace is {
bool input(const uint8_t& index) {
  return configuration[index] & DigitalInput;
}
namespace digital {
bool output(const uint8_t& index) {
  return configuration[index] == DigitalOutput;
}
}
namespace analog {
bool output(const uint8_t& index) {
  return configuration[index] == AnalogOutput;
}
}
}
}

}
}
