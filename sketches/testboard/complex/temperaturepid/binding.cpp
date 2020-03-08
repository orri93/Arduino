#include "binding.h"
#include "variable.h"
#include "pid.h"

namespace gt = ::gos::temperature;
namespace gtvt = ::gos::temperature::variables::temporary;

namespace gatl = ::gos::atl;

namespace gos {
namespace temperature {
namespace binding {

void create() {
  gtvt::integer = gatl::binding::create<gt::type::Unsigned, uint16_t, uint8_t>(
    gt::binding::modbus::input::registers::uints,   // Reference
    0x0000,                                         // Address
    0x01,                                           // Count
    sizeof(gt::type::Unsigned) / 2);                // Size
  gatl::binding::set<gt::type::Unsigned, uint16_t, uint8_t>(
    gt::binding::modbus::input::registers::uints,   // Reference
    0x00,                                           // Index
    &gt::variables::output);                        // Variable pointer

  gtvt::integer = gatl::binding::create<gt::type::Real, uint16_t, uint8_t>(
    gt::binding::modbus::input::registers::real,    // Reference
    gtvt::integer,                                  // Address
    0x01,                                           // Count
    sizeof(gt::type::Real) / 2);                    // Size
  gatl::binding::set<gt::type::Real, uint16_t, uint8_t>(
    gt::binding::modbus::input::registers::real,    // Reference
    0x00,                                           // Index
    &gt::variables::temperature);                   // Variable pointer

  gtvt::integer = gatl::binding::create<gt::type::Unsigned, uint16_t, uint8_t>(
    gt::binding::modbus::holding::registers::uints, // Reference
    0x0000,                                         // Address
    0x02,                                           // Count
    sizeof(gt::type::Unsigned) / 2);                // Size
  gatl::binding::set<gt::type::Unsigned, uint16_t, uint8_t>(
    gt::binding::modbus::holding::registers::uints, // Reference
    0x00,                                           // Index
    &gt::variables::interval);                      // Variable pointer
  gatl::binding::set<gt::type::Unsigned, uint16_t, uint8_t>(
    gt::binding::modbus::holding::registers::uints, // Reference
    0x01,                                           // Index
    &gt::variables::manual);                        // Variable pointer

  gtvt::integer = gatl::binding::create<gt::type::Real, uint16_t, uint8_t>(
    gt::binding::modbus::holding::registers::real,  // Reference
    gtvt::integer,                                  // Address
    0x02,                                           // Count
    sizeof(gt::type::Real) / 2);                    // Size
  gatl::binding::set<gt::type::Real, uint16_t, uint8_t>(
    gt::binding::modbus::holding::registers::real,  // Reference
    0x00,                                           // Index
    &(gt::pid::parameter.Setpoint));                // Variable pointer
  gatl::binding::set<gt::type::Real, uint16_t, uint8_t>(
    gt::binding::modbus::holding::registers::real,  // Reference
    0x01,                                           // Index
    &gt::pid::parameter.Kp);                        // Variable pointer
}

namespace modbus {

namespace input {
namespace registers {
Unsigned uints;
Real real;
} // namespace registers
} // namespace input

namespace holding {
namespace registers {
Unsigned uints;
Real real;
} // namespace registers
} // namespace holding

} // namespace modbus

} // namespace display
} // namespace temperature
} // namespace gos
