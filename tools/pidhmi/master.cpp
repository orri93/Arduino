#include <iostream>
#include <string>
#include <regex>

#include <modbus.h>

#include <master.h>

#define GOS_DL_PR_COILS 1
#define GOS_DL_PR_DISCRETE 1
#define GOS_DL_PR_HOLDING 9
#define GOS_DL_PR_INPUT 3

namespace gos {
namespace arduino {
namespace tools {
namespace pid {
namespace master {

namespace report {
namespace error {
errno_t _no;
std::string _last;
errno_t errorno() {
  return _no;
}
std::string last() {
  return _last;
}
}
}

namespace details {
uint8_t _coils[GOS_DL_PR_COILS];
uint8_t _discrete[GOS_DL_PR_DISCRETE];
uint16_t _registers[GOS_DL_PR_HOLDING];
uint16_t _registersout[GOS_DL_PR_HOLDING];
uint16_t _input[GOS_DL_PR_INPUT];
modbus_t* _modbus = nullptr;
}

result initialize(const char* device, const int& baud, const int& slaveid) {
  std::regex highport("COM\\d{2}");
  int result;
  std::string port = device;
  if (std::regex_match(port, highport)) {
    port = "\\\\.\\" + port;
  }
  details::_modbus = modbus_new_rtu(port.c_str(), baud, 'N', 8, 1);
  if (details::_modbus != nullptr) {
    result = modbus_set_slave(details::_modbus, slaveid);
    return result::success;
  } else {
    return result::failure;
  }
}

result connect() {
  int result;
  if (details::_modbus) {
    result = modbus_connect(details::_modbus);
    if (result >= 0) {
      return result::success;
    } else {
      report::error::_no = errno;
      report::error::_last = modbus_strerror(report::error::_no);
      return result::failure;
    }
  } else {
    return result::uninitialized;
  }
}

result disconnect() {
  if (details::_modbus) {
    ::modbus_close(details::_modbus);
    return result::success;
  } else {
    return result::uninitialized;
  }
}

result shutdown() {
  if (details::_modbus) {
    ::modbus_free(details::_modbus);
    details::_modbus = nullptr;
    return result::success;
  } else {
    return result::uninitialized;
  }
}

namespace read {
result tunecoil(bool& output) {
  if (details::_modbus) {
    int result = ::modbus_read_bits(
      details::_modbus,
      0,
      1,
      details::_coils);
    if (result >= 0) {
      output = details::_coils[0] != 0;
      return result::success;
    } else {
      return result::failure;
    }
  } else {
    return result::uninitialized;
  }
}
result inputs(uint16_t& output, float& temperature) {
  if (details::_modbus) {
    int result = ::modbus_read_input_registers(
      details::_modbus,
      0,
      3,
      details::_input);
    if (result >= 0) {
      output = details::_input[0];
      temperature = modbus_get_float(details::_input + 1);
      return result::success;
    } else {
      return result::failure;
    }
  } else {
    return result::uninitialized;
  }
}
result holding(
  uint16_t& manual,
  float& setpoint,
  float& kp,
  float& ki,
  float& kd,
  float& ti,
  float& td) {
  if (details::_modbus) {
    int result = ::modbus_read_registers(
      details::_modbus,
      0,
      13,
      details::_registers);
    if (result >= 0) {
      manual = details::_registers[0];
      uint16_t* pointer = details::_registers + 1;
      setpoint = ::modbus_get_float(pointer);
      pointer += 2;
      kp = ::modbus_get_float(pointer);
      pointer += 2;
      ki = ::modbus_get_float(pointer);
      pointer += 2;
      kd = ::modbus_get_float(pointer);
      pointer += 2;
      ti = ::modbus_get_float(pointer);
      pointer += 2;
      td = ::modbus_get_float(pointer);
      return result::success;
    } else {
      return result::failure;
    }
  } else {
    return result::uninitialized;
  }
}
}

namespace set {
result tunecoil(const bool& output) {
  if (details::_modbus) {
    int result = ::modbus_write_bit(details::_modbus, 0, output ? 1 : 0);
    if (result >= 0) {
      return result::success;
    } else {
      return result::failure;
    }
  } else {
    return result::uninitialized;
  }
}
result manual(const uint16_t& manual) {
  if (details::_modbus) {
    int result = ::modbus_write_register(details::_modbus, 0, manual);
    if (result >= 0) {
      std::cout << "Writing manual value " << manual
        << " to address 0 was successful" << std::endl;
      return result::success;
    } else {
      std::cout << "Writing manual value " << manual
        << " to address 0 failed" << std::endl;
      return result::failure;
    }
  } else {
    return result::uninitialized;
  }
}

result real(const float& value, const int& address) {
  if (details::_modbus) {
    ::modbus_set_float(value, details::_registers);
    int result = ::modbus_write_registers(
      details::_modbus,
      address,
      2,
      details::_registers);
    if (result >= 0) {
      std::cout << "Writing " << value << " to address "
        << address << " was successful" << std::endl;
      return result::success;
    } else {
      std::cerr << "Failed to writing " << value << " to address "
        << address << std::endl;
      return result::failure;
    }
  } else {
    return result::uninitialized;
  }
}

result setpoint(const float& setpoint) {
  return real(setpoint, 1);
}

result kp(const float& kp) {
  return real(kp, 3);
}
result ki(const float& ki) {
  return real(ki, 5);
}
result kd(const float& kd) {
  return real(kd, 7);
}
result ti(const float& ti) {
  return real(ti, 9);
}
result td(const float& td) {
  return real(td, 11);
}
}

}

}
}
}
}
