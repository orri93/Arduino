#ifndef GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_MODBUS_BINDING_H_
#define GOS_ARDUINO_SKETCHES_TESTBOARD_COMPLEX_MODBUS_BINDING_H_

#include <gatlmodbus.h>
#include <gatlbinding.h>

#include "macro.h"
#include "type.h"

namespace gos {
namespace modbus {

namespace binding {
#ifdef GOS_BARRAY_BINDING
namespace barray {
typedef ::gos::atl::binding::barray::reference<
  type::Real, MODBUS_TYPE_DEFAULT, MODBUS_TYPE_BIND_INDEX> RealBinding;
extern RealBinding real;
}
#else
typedef ::gos::atl::binding::reference<type::Real, uint16_t> RealBinding;
extern RealBinding real;
#endif
void create();
} // namespace binding

} // namespace modbus
} // namespace gos

#endif
