#include <cassert>

#include <iostream>

#include <gos/arduino/test/tools/modbus/gam.h>

#define MEMORY_SIZE 512

#ifndef lowByte
#define lowByte(w) ((uint8_t) ((w) & 0xff))
#endif
#ifndef highByte
#define highByte(w) ((uint8_t) ((w) >> 8))
#endif

namespace gatt = ::gos::arduino::test::tools;

namespace ga = ::gos::atl;
namespace gatlm = ::gos::atl::modbus;
namespace gau = ::gos::atl::utility;
namespace gaur = ::gos::atl::utility::range;

namespace gos {
namespace arduino {
namespace test {
namespace tools {
namespace modbus {

gam::gam(Stream& stream, const int& id, const size_t& size) :
  stream_(stream)
#ifdef MODBUS_HANDLER_INTERFACE
  ,handler_(id, size)
#else
  ,id_(id)
  ,size_(size)
#endif
{
}

void gam::begin(const int& baud) {
#ifdef MODBUS_HANDLER_INTERFACE
  bool r = handler_.create();
  assert(r);
  gatlm::begin<Type>(stream_, handler_.parameter_, handler_.variable_, baud);
#else
  bool r = initialize(id_, size_);
  assert(r);
  gatlm::begin<Type>(stream_, _parameter, _variable, baud);
#endif
}

void gam::loop() {
#ifdef MODBUS_HANDLER_INTERFACE
  Type lr = gatlm::loop<Type>(
    stream_,
    handler_.parameter_,
    handler_,
    handler_.variable_,
    handler_.request_,
    handler_.response_);
#else
  Type lr = gatlm::loop<Type>(
    stream_,
    _parameter,
    _variable,
    *_request,
    *_response);
#endif
}

#ifdef MODBUS_HANDLER_INTERFACE
Handler::Handler(const int& id, const size_t& size) :
  engine_(random_()),
  distribution_(0, 0xffff),
  memory_(MEMORY_SIZE),
  request_(static_cast<Type>(size)),
  response_(static_cast<Type>(size)),
  coils_(0) {
  parameter_.Control = 0;
  parameter_.Id = id;
  variable_.Index.Write = 0;
  variable_.Length.Request = 0;
  variable_.Length.Response = 0;
  variable_.Length.Transmission = 0;
  variable_.Reading = false;
  variable_.Writing = false;
  variable_.Time.Half = 0;
  variable_.Time.Last = 0;
}

bool Handler::create() {
  MODBUS_TYPE_BUFFER* p = memory_.create();
  return p != nullptr;
}

MODBUS_TYPE_RESULT Handler::ReadCoils(
  const Type& address,
  const Type& length) {
  std::cout << "GOS Modbus Reading " << length
    << " coils from address " << address << std::endl;
  for (MODBUS_TYPE_DEFAULT i = 0; i < length; ++i) {
    if (address + i < 8) {
      gatlm::provide::coil(
        variable_,
        request_,
        response_,
        i,
        bitRead(coils_, address + i));
    } else {
      return MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
    }
  }
  return MODBUS_STATUS_OK;
}

MODBUS_TYPE_RESULT Handler::ReadDiscreteInputs(
  const Type& address,
  const Type& length) {
  MODBUS_TYPE_RESULT result = MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;

  std::cout << "GOS Modbus Reading " << length
    << " discrete inputs from address " << address << std::endl;
  for (Type i = 2; i < length; ++i) {
    gatlm::provide::discrete(variable_, request_, response_, i, true);
  }

  if (gaur::ismemberof<uint16_t>(0x0000, address, length)) {
    std::cout << "GOS Modbus Reading " << length
      << " discrete inputs number " << 0x0000 << std::endl;
    if ((result = gatlm::provide::discrete<Type>(
      variable_, request_, response_, 0x0000, true)) != MODBUS_STATUS_OK) {
      return result;
    }
  }
  if (gaur::ismemberof<uint16_t>(0x0001, address, length)) {
    std::cout << "GOS Modbus Reading " << length
      << " discrete inputs number " << 0x0001 << std::endl;
    if ((result = gatlm::provide::discrete<Type>(
      variable_, request_, response_, 0x0001, true)) != MODBUS_STATUS_OK) {
      return result;
    }
  }

  return result;
}

MODBUS_TYPE_RESULT Handler::ReadHoldingRegisters(
  const Type& address,
  const Type& length) {
  for (MODBUS_TYPE_DEFAULT i = 0; i < length; ++i) {
    if (address + i < 256) {
      uint8_t lb = memory_.read(2 * (static_cast<size_t>(address) + i));
      uint8_t hb = memory_.read(2 * (static_cast<size_t>(address) + i) + 1);
      gatlm::provide::registers<>(
        variable_,
        request_,
        response_,
        i,
        word(hb, lb));
    } else {
      return MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
    }
  }
  return MODBUS_STATUS_OK;
}

MODBUS_TYPE_RESULT Handler::ReadInputRegisters(
  const Type& address,
  const Type& length) {
  for (MODBUS_TYPE_DEFAULT i = 0; i < length; ++i) {
    if (address + i < 256) {
      uint16_t w = distribution_(engine_);
      gatlm::provide::registers<>(variable_, request_, response_, i, w);
    } else {
      return MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
    }
  }
  return MODBUS_STATUS_OK;
}

MODBUS_TYPE_RESULT Handler::WriteCoils(
  const MODBUS_TYPE_FUNCTION& function,
  const Type& address,
  const Type& length) {
  for (MODBUS_TYPE_DEFAULT i = 0; i < length; ++i) {
    if (address + i < 8) {
      if (gatlm::access::coil(variable_, request_, i)) {
        bitSet(coils_, address + i);
      } else {
        bitClear(coils_, address + i);
      }
    } else {
      return MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
    }
  }
  return MODBUS_STATUS_OK;
}

MODBUS_TYPE_RESULT Handler::WriteHoldingRegisters(
  const MODBUS_TYPE_FUNCTION& function,
  const Type& address,
  const Type& length) {
  for (Type i = 0; i < length; ++i) {
    if (address + i < 256) {
      MODBUS_TYPE_DEFAULT r = gatlm::access::registers<>(
        variable_,
        request_,
        i);
      memory_.update(2 * (static_cast<size_t>(address) + i), lowByte(r));
      memory_.update(2 * (static_cast<size_t>(address) + i) + 1, highByte(r));
    } else {
      return MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
    }
  }
  return MODBUS_STATUS_OK;
}

MODBUS_TYPE_RESULT Handler::ReadExceptionStatus() {
  return MODBUS_STATUS_OK;
}
#else
Device _random;
Engine _engine(_random());
Distribution _distribution(0, 0xffff);

Memory _memory(MEMORY_SIZE);
Parameter _parameter;
Variable _variable;
BufferPointer _request;
BufferPointer _response;
uint8_t _coils = 0;

bool initialize(const int& id, const size_t& size) {
  
  _request = std::make_unique<Buffer>(static_cast<Type>(size));
  _response = std::make_unique<Buffer>(static_cast<Type>(size));
  
  _parameter.Control = 0;
  _parameter.Id = id;
  _variable.Index.Write = 0;
  _variable.Length.Request = 0;
  _variable.Length.Response = 0;
  _variable.Length.Transmission = 0;
  _variable.Reading = false;
  _variable.Writing = false;
  _variable.Time.Half = 0;
  _variable.Time.Last = 0;

  gatlm::callback::set::read::coils(&callback::read::coils);
  gatlm::callback::set::write::coils(&callback::write::coils);

  MODBUS_TYPE_BUFFER* p = _memory.create();
  return p != nullptr;
}

namespace callback {
namespace read {
MODBUS_TYPE_RESULT coils(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  std::cout << "GOS Modbus Reading " << length
    << " coils from address " << address << std::endl;
  for (MODBUS_TYPE_DEFAULT i = 0; i < length; ++i) {
    if (address + i < 8) {
      gatlm::provide::coil<>(
        _variable,
        *_request,
        *_response,
        i,
        bitRead(_coils, address + i));
    } else {
      return MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
    }
  }
  return MODBUS_STATUS_OK;
}
namespace discrete {
static MODBUS_TYPE_RESULT inputs(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  return MODBUS_STATUS_OK;
}
}
namespace holding {
static MODBUS_TYPE_RESULT registers(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  return MODBUS_STATUS_OK;
}
}
namespace input {
static MODBUS_TYPE_RESULT registers(
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  return MODBUS_STATUS_OK;
}
}
namespace exception {
static MODBUS_TYPE_RESULT status() {
  return MODBUS_STATUS_OK;
}
}
}
namespace write {
static MODBUS_TYPE_RESULT coils(
  const MODBUS_TYPE_FUNCTION& function,
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  for (MODBUS_TYPE_DEFAULT i = 0; i < length; ++i) {
    if (address + i < 8) {
      if (gatlm::access::coil<>(_variable, *_request, i)) {
        bitSet(_coils, address + i);
      } else {
        bitClear(_coils, address + i);
      }
    } else {
      return MODBUS_STATUS_ILLEGAL_DATA_ADDRESS;
    }
  }
  return MODBUS_STATUS_OK;
}
namespace holding {
static MODBUS_TYPE_RESULT registers(
  const MODBUS_TYPE_FUNCTION& function,
  const MODBUS_TYPE_DEFAULT& address,
  const MODBUS_TYPE_DEFAULT& length) {
  return MODBUS_STATUS_OK;
}
}
}
}
#endif


} // namespace modbus
} // namespace tools
} // namespace test
} // namespace arduino 
} // namespace gos
