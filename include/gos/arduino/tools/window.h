#ifndef GOS_ARDUINO_TOOL_WINDOW_H_
#define GOS_ARDUINO_TOOL_WINDOW_H_

#include <cmath>

#include <memory>
#include <vector>

#include <gos/arduino/tools/statistics.h>

namespace gos {
namespace arduino {
namespace tools {
namespace statistics {

template<typename T>
class window : public virtual ::gos::arduino::tools::Statistics<T> {
public:

  window() : size_(10) {
  }

  window(const size_t& size) : size_(size) {
  }

  void set(const size_t& size) {
    size_ = size;
  }

  const size_t& size() const {
    return size_;
  }

  void add(const T& value) {
    if (static_cast<size_t>(this->vector_.size()) >= this->size_) {
      this->sum_ -= *(this->vector_.begin());
      this->vector_.erase(this->vector_.begin());
    }
    ::gos::arduino::tools::Statistics<T>::add(value);
  }

private:
  size_t size_;
};

} // namespace statistics
} // namespace tools
} // namespace arduino 
} // namespace gos
#endif
