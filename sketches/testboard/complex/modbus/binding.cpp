#include "binding.h"

namespace gos {
namespace modbus {

namespace binding {
#ifdef GOS_BARRAY_BINDING
namespace barray {
RealBinding real;
} // namespace barray
#else
RealBinding real;
#endif

void create() {
}

} // namespace binding

} // namespace modbus
} // namespace gos
