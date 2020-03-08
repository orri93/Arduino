#include <Arduino.h>
#include <EEPROM.h>

#include "macro.h"
#include "variable.h"

#ifndef NO_DISPLAY
#include "format.h"
#include "display.h"
#endif

#include "modbus.h"

#ifndef USE_ARDUINO_MODBUS_SLAVE
namespace gatl = ::gos::atl;
namespace gatlu = ::gos::atl::utility;
#endif

namespace gm = ::gos::modbus;

#ifndef NO_DISPLAY
namespace gmd = ::gos::modbus::display;
#endif

namespace gos {
namespace modbus {

#ifdef USE_ARDUINO_MODBUS_SLAVE
Modbus slave(1, PIN_RS485_MODBUS_TE);
uint8_t ReadCoils(uint8_t function, uint16_t start, uint16_t length) {
  uint8_t result = STATUS_OK;
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  for (uint16_t i = 0; i < length; ++i) {
    if (start + i < 8) {
      slave.writeCoilToBuffer(i, bitRead(gm::variables::coils, start + i));
    } else {
      result = STATUS_ILLEGAL_DATA_ADDRESS;
      break;
    }
  }
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return result;
}
uint8_t ReadDiscreteInputs(uint8_t function, uint16_t start, uint16_t length) {
  uint8_t result = STATUS_OK;
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  if (start == 0 && length == 1) {
    slave.writeDiscreteInputToBuffer(0, analogRead(PIN_BUTTON) < 512);
  } else {
    result = STATUS_ILLEGAL_DATA_ADDRESS;
  }
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return result;
}
uint8_t ReadHoldingRegisters(uint8_t function, uint16_t start, uint16_t length) {
  uint16_t reg;
  uint8_t hb, lb;
  uint8_t result = STATUS_OK;
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  for (uint16_t i = 0; i < length; ++i) {
    if (start + i < 8) {
      lb = EEPROM.read(2 * (start + i));
      hb = EEPROM.read(1 + 2 * (start + i));
      reg = word(hb, lb);
      slave.writeRegisterToBuffer(i, word(hb, lb));
    } else {
      result = STATUS_ILLEGAL_DATA_ADDRESS;
      break;
    }
  }
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return result;
}
uint8_t ReadInputRegisters(uint8_t function, uint16_t start, uint16_t length) {
  uint8_t result = STATUS_OK;
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  if (start == 0 && length == 1) {
    slave.writeRegisterToBuffer(0, analogRead(A2));
  } else {
    result = STATUS_ILLEGAL_DATA_ADDRESS;
  }
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return result;
}
uint8_t WriteCoils(uint8_t function, uint16_t start, uint16_t length) {
  uint8_t result = STATUS_OK;
  digitalWrite(PIN_LED_MODBUS_WRITE, HIGH);
  for (uint16_t i = 0; i < length; ++i) {
    if (start + i < 8) {
      if(slave.readCoilFromBuffer(i)) {
        bitSet(gm::variables::coils, start + i);
      } else {
        bitClear(gm::variables::coils, start + i);
      }
    } else {
      result = STATUS_ILLEGAL_DATA_ADDRESS;
      break;
    }
  }
  digitalWrite(PIN_LED_MODBUS_WRITE, LOW);
  return result;
}
uint8_t WriteHoldingRegisters(uint8_t function, uint16_t start, uint16_t length) {
  uint16_t reg;
  uint8_t hb, lb;
  uint8_t result = STATUS_OK;
  digitalWrite(PIN_LED_MODBUS_WRITE, HIGH);
  for (uint16_t i = 0; i < length; ++i) {
    if (start + i < 8) {
      reg = slave.readRegisterFromBuffer(i);
      lb = lowByte(reg);
      hb = highByte(reg);
      EEPROM.update(2 * (start + i), lb);
      EEPROM.update(1 + 2 * (start + i), hb);
      if (start + i == 0) {
        gm::variables::led::blue::value = lb;
      }
    } else {
      result = STATUS_ILLEGAL_DATA_ADDRESS;
      break;
    }
  }
  digitalWrite(PIN_LED_MODBUS_WRITE, LOW);
  return result;
}
void initialize() {
  slave.cbVector[CB_READ_COILS] = &ReadCoils;
  slave.cbVector[CB_READ_DISCRETE_INPUTS] = &ReadDiscreteInputs;
  slave.cbVector[CB_READ_HOLDING_REGISTERS] = &ReadHoldingRegisters;
  slave.cbVector[CB_READ_INPUT_REGISTERS] = &ReadInputRegisters;
  slave.cbVector[CB_WRITE_COILS] = &WriteCoils;
  slave.cbVector[CB_WRITE_HOLDING_REGISTERS] = &WriteHoldingRegisters;
  slave.begin(MODBUS_BAUD);
}
#else
namespace buffer {
Holder request(MODBUS_BUFFER_SIZE);
Holder response(MODBUS_BUFFER_SIZE);
} // namespace buffer

Parameter parameter;
Variable variable;

#ifdef MODBUS_HANDLER_INTERFACE
void initialize() {
  parameter.Id = 1;
  parameter.Control = PIN_RS485_MODBUS_TE;
  gatl::modbus::begin<>(Serial, parameter, variable, MODBUS_BAUD);
}

Handler handler;

/* 0x01 Read Coils */
MODBUS_TYPE_RESULT gm::Handler::ReadCoils(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  MODBUS_TYPE_RESULT result = MODBUS_STATUS_OK;
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  for (MODBUS_TYPE_DEFAULT i = 0; i < length; ++i) {
    if (address + i < 8) {
      gatl::modbus::provide::coil<>(
        gm::variable,
        gm::buffer::request,
        gm::buffer::response,
        i,
        bitRead(gm::variables::coils, address + i));
    } else {
      result = MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
      break;
    }
  }
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return result;
}

/* 0x02 Read Discretes */
MODBUS_TYPE_RESULT gm::Handler::ReadDiscreteInputs(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  MODBUS_TYPE_RESULT result = MODBUS_STATUS_OK;
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  if (address == 0 && length == 1) {
    gatl::modbus::provide::discrete<MODBUS_TYPE_DEFAULT>(
      gm::variable,
      gm::buffer::request,
      gm::buffer::response,
      0,
      analogRead(PIN_BUTTON) < 512);
  } else {
    result = MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  }
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return result;
}

/* 0x03 Read Multiple Holding Registers */
MODBUS_TYPE_RESULT gm::Handler::ReadHoldingRegisters(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  uint8_t hb, lb;
  MODBUS_TYPE_DEFAULT reg;
  MODBUS_TYPE_RESULT result = MODBUS_STATUS_OK;
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  for (MODBUS_TYPE_DEFAULT i = 0; i < length; ++i) {
    if (address + i < 8) {
      lb = EEPROM.read(2 * (address + i));
      hb = EEPROM.read(1 + 2 * (address + i));
      reg = word(hb, lb);
      gatl::modbus::provide::registers<>(
        gm::variable,
        gm::buffer::request,
        gm::buffer::response,
        i,
        reg);
    } else {
      result = MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
      break;
    }
  }
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return result;
}

/* 0x04 Read Input Registers */
MODBUS_TYPE_RESULT gm::Handler::ReadInputRegisters(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  MODBUS_TYPE_RESULT result = MODBUS_STATUS_OK;
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  if (address == 0 && length == 1) {
    gatl::modbus::provide::registers<MODBUS_TYPE_DEFAULT>(
      gm::variable,
      gm::buffer::request,
      gm::buffer::response,
      0,
      analogRead(A2));
  } else {
    result = MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  }
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return result;
}

/* 0x05 Write Single Coil and 0x0f Write Multiple Coils */
MODBUS_TYPE_RESULT gm::Handler::WriteCoils(
  const MODBUS_TYPE_FUNCTION& function,
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  MODBUS_TYPE_RESULT result = MODBUS_STATUS_OK;
  digitalWrite(PIN_LED_MODBUS_WRITE, HIGH);
  for (MODBUS_TYPE_DEFAULT i = 0; i < length; ++i) {
    if (address + i < 8) {
      if (gatl::modbus::access::coil<>(gm::variable, gm::buffer::request, i)) {
        bitSet(gm::variables::coils, address + i);
      } else {
        bitClear(gm::variables::coils, address + i);
      }
    } else {
      result = MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
      break;
    }
  }
  digitalWrite(PIN_LED_MODBUS_WRITE, LOW);
  return result;
}

/* 0x06 Write Single and 0x10 Write Multiple Holding Registers */
MODBUS_TYPE_RESULT gm::Handler::WriteHoldingRegisters(
  const MODBUS_TYPE_FUNCTION& function,
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  uint8_t hb, lb;
  MODBUS_TYPE_DEFAULT reg;
  MODBUS_TYPE_RESULT result = MODBUS_STATUS_OK;
  digitalWrite(PIN_LED_MODBUS_WRITE, HIGH);
  for (MODBUS_TYPE_DEFAULT i = 0; i < length; ++i) {
    if (address + i < 8) {
      reg = gatl::modbus::access::registers<>(
        gm::variable,
        gm::buffer::request,
        i);
      lb = lowByte(reg);
      hb = highByte(reg);
      EEPROM.update(2 * (address + i), lb);
      EEPROM.update(1 + 2 * (address + i), hb);
      if (address + i == 0) {
        gm::variables::led::blue::value = lb;
      }
    } else {
      result = MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
      break;
    }
  }
  digitalWrite(PIN_LED_MODBUS_WRITE, LOW);
  return result;
}

MODBUS_TYPE_RESULT gm::Handler::ReadExceptionStatus() {
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return MODBUS_STATUS_OK;
}
#else
void initialize() {
  parameter.Id = MODBUS_SLAVE_ID;
  parameter.Control = PIN_RS485_MODBUS_TE;

  gatl::modbus::callback::set::read::coils(
    ::gos::modbus::callback::read::coils);
  gatl::modbus::callback::set::read::discrete::inputs(
    ::gos::modbus::callback::read::discrete::inputs);
  gatl::modbus::callback::set::read::input::registers(
    ::gos::modbus::callback::read::input::registers);
  gatl::modbus::callback::set::read::holding::registers(
    ::gos::modbus::callback::read::holding::registers);
  gatl::modbus::callback::set::write::coils(
    ::gos::modbus::callback::write::coils);
  gatl::modbus::callback::set::write::holding::registers(
    ::gos::modbus::callback::write::holding::registers);

  gatl::modbus::begin<>(Serial, parameter, variable, MODBUS_BAUD);
}
namespace callback {
namespace read {
MODBUS_TYPE_RESULT coils(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  MODBUS_TYPE_RESULT result = MODBUS_STATUS_OK;
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  for (MODBUS_TYPE_DEFAULT i = 0; i < length; ++i) {
    if (address + i < 8) {
      gatl::modbus::provide::coil<>(
        gm::variable,
        gm::buffer::request,
        gm::buffer::response,
        i,
        bitRead(gm::variables::coils, address + i));
    } else {
      result = MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
      break;
    }
  }
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return result;
}
namespace discrete {
MODBUS_TYPE_RESULT inputs(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  MODBUS_TYPE_RESULT result = MODBUS_STATUS_OK;
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  if (address == 0 && length == 1) {
    gatl::modbus::provide::discrete<MODBUS_TYPE_DEFAULT>(
      gm::variable,
      gm::buffer::request,
      gm::buffer::response,
      0,
      analogRead(PIN_BUTTON) < 512);
  } else {
    result = MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  }
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return result;
}
}
namespace holding {
MODBUS_TYPE_RESULT registers(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  uint8_t hb, lb;
  MODBUS_TYPE_DEFAULT reg;
  MODBUS_TYPE_RESULT result = MODBUS_STATUS_OK;
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  for (MODBUS_TYPE_DEFAULT i = 0; i < length; ++i) {
    if (address + i < 8) {
      lb = EEPROM.read(2 * (address + i));
      hb = EEPROM.read(1 + 2 * (address + i));
      reg = word(hb, lb);
      gatl::modbus::provide::registers<>(
        gm::variable,
        gm::buffer::request,
        gm::buffer::response,
        i,
        reg);
    } else {
      result = MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
      break;
    }
  }
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return result;
}
}
namespace input {
MODBUS_TYPE_RESULT registers(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  MODBUS_TYPE_RESULT result = MODBUS_STATUS_OK;
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  if (address == 0 && length == 1) {
    gatl::modbus::provide::registers<MODBUS_TYPE_DEFAULT>(
      gm::variable,
      gm::buffer::request,
      gm::buffer::response,
      0,
      analogRead(A2));
  } else {
    result = MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
  }
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return result;
}
}
namespace exception {
MODBUS_TYPE_RESULT status() {
  digitalWrite(PIN_LED_MODBUS_READ, HIGH);
  digitalWrite(PIN_LED_MODBUS_READ, LOW);
  return MODBUS_STATUS_OK;
}
}
}
namespace write {
MODBUS_TYPE_RESULT coils(
  const MODBUS_TYPE_FUNCTION& function,
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  MODBUS_TYPE_RESULT result = MODBUS_STATUS_OK;
  digitalWrite(PIN_LED_MODBUS_WRITE, HIGH);
  for (MODBUS_TYPE_DEFAULT i = 0; i < length; ++i) {
    if (address + i < 8) {
      if (gatl::modbus::access::coil<>(gm::variable, gm::buffer::request, i)) {
        bitSet(gm::variables::coils, address + i);
      } else {
        bitClear(gm::variables::coils, address + i);
      }
    } else {
      result = MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
      break;
    }
  }
  digitalWrite(PIN_LED_MODBUS_WRITE, LOW);
  return result;
}
namespace holding {
MODBUS_TYPE_RESULT registers(
  const MODBUS_TYPE_FUNCTION& function,
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  uint8_t hb, lb;
  MODBUS_TYPE_DEFAULT reg;
  MODBUS_TYPE_RESULT result = MODBUS_STATUS_OK;
  digitalWrite(PIN_LED_MODBUS_WRITE, HIGH);
  for (MODBUS_TYPE_DEFAULT i = 0; i < length; ++i) {
    if (address + i < 8) {
      reg = gatl::modbus::access::registers<>(
        gm::variable,
        gm::buffer::request,
        i);
      lb = lowByte(reg);
      hb = highByte(reg);
      EEPROM.update(2 * (address + i), lb);
      EEPROM.update(1 + 2 * (address + i), hb);
      if (address + i == 0) {
        gm::variables::led::blue::value = lb;
      }
    } else {
      result = MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
      break;
    }
  }
  digitalWrite(PIN_LED_MODBUS_WRITE, LOW);
  return result;
}
}
}
}
#endif
#endif

} // namespace modbus
} // namespace gos
