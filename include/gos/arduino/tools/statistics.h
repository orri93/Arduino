#ifndef GOS_ARDUINO_TOOL_STATISTICS_H_
#define GOS_ARDUINO_TOOL_STATISTICS_H_

#include <cmath>

#include <memory>
#include <vector>

#include <gos/arduino/tools/types.h>

namespace gos {
namespace arduino {
namespace tools {

template<typename T>
class Statistics {
public:
  typedef ::std::vector<T> Vector;

  Statistics() : sum_(0) {
  }

  virtual ~Statistics() {
  }

  void setrange(const double& lowest, const double& highest) {
    range_ = std::make_unique<::gos::arduino::tools::types::range<T>>(
      lowest,
      highest);
  }

  virtual void add(const T& value) {
    this->vector_.push_back(value);
    this->sum_ += value;
  }

  void clear() {
    this->sum_ = T(0);
    this->vector_.clear();
  }

  size_t count() const {
    return static_cast<size_t>(vector_.size());
  }

  const Vector& vector() const {
    return vector_;
  }

  const T& sum() const {
    return sum_;
  }

  T mean() const {
    return sum_ / static_cast<T>(vector_.size());
  }

  T median() {
    VectorSizeType size = this->vector_.size();
    if (size > 1) {
      size_t medianindex = size / static_cast<T>(2);
      Vector sorted(vector_);
      ::std::sort(sorted.begin(), sorted.end());
      if (size % 2 == 0) {
        return (sorted[medianindex - 1] + sorted[medianindex]) /
          static_cast<T>(2);
      } else {
        return sorted[medianindex];
      }
    } else if (size == 1) {
      return *(this->vector_.begin());
    } else {
      return 0.0;
    }
  }

  T variance() const {
    T diff;
    T variance = 0.0;
    T mean = this->mean();
    for (auto v : this->vector_) {
      diff = v - mean;
      variance += diff * diff;
    }
    return variance / this->vector_.size();
  }

  T sd() const {
    if (T == double) {
      return ::sqrt(variance());
    } else if (T == float) {
      return ::sqrtf(variance())
    } else {
      return static_cast<T>(::sqrt(static_cast<double>(variance())));
    }
  }

protected:
  typedef std::unique_ptr<::gos::arduino::tools::types::range<T>> RangePointer;
  Vector vector_;
  T sum_;
private:
  RangePointer range_;
};

} // namespace tools
} // namespace arduino 
} // namespace gos
#endif
