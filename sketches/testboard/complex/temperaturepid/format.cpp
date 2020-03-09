#include "format.h"
#include "text.h"

#define GOS_TEXT_BUFFER(n,t) Holder n(t,sizeof(t))

namespace gos {
namespace temperature {

namespace format {
namespace display {
namespace buffer {
Holder first;
Holder second;
namespace text {
GOS_TEXT_BUFFER(interval, GOS_TCT_INTERVAL);
GOS_TEXT_BUFFER(manual, GOS_TCT_MANUAL);
} // namespace text
} // namespace buffer
} // namespace display
} // namespace format

} // namespace temperature
} // namespace gos
