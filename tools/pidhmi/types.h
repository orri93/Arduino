#ifndef TYPES_H
#define TYPES_H

namespace gos {
namespace arduino {
namespace tools {
namespace pid {

enum class status {
  undefined,
  idle,
  connecting,
  connected,
  disconnecting,
  down };

}
}
}
}

#endif
