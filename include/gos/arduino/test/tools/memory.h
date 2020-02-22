#ifndef GOS_ARDUINO_TEST_TOOL_MODBUS_MEMORY_H_
#define GOS_ARDUINO_TEST_TOOL_MODBUS_MEMORY_H_

#include <memory>

namespace gos {
namespace arduino {
namespace test {
namespace tools {

template<typename T> class Memory {
public:
  Memory(const size_t& size) : size_(size) {
  }

  T* create() {
    buffer_ = std::make_unique<T[]>(size_);
    if (buffer_) {
      return buffer_.get();
    } else {
      return nullptr;
    }
  }

  void update(const size_t& address, const T& value) {
    if (((bool)buffer_) && address < size_) {
      buffer_[address] = value;
    }
  }

  T read(const size_t& address) {
    if (((bool)buffer_) && address < size_) {
      return buffer_[address];
    } else {
      return T();
    }
  }

private:
  typedef std::unique_ptr<T[]> Buffer;
  Buffer buffer_;
  size_t size_;
};

}
}
}
}

#endif
