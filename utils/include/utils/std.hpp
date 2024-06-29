#ifndef UTILS__STD_HPP
#define UTILS__STD_HPP

#include <Eigen/Core>

#include "logging.hpp"

namespace std {

void sleep(double seconds);


// std::vector -> Eigen
template<typename T>
Eigen::Vector<T, -1> toEigen(const vector<T> &vec) {
  Eigen::Vector<T, -1> v(vec.size());
  for (int i = 0; i < vec.size(); i++) v(i) = vec[i];
  return v;
}


// 按值切片器
template<typename T>
class ValueSlicer {
    int mIndex = 0;
    const std::vector<T> mValues;
    std::function<bool(const T &, const T &)> mCompare;

public:
    explicit ValueSlicer(const std::vector<T> &values, bool ascending = true
    ) : mValues(values), mCompare(ascending ? [](const T &a, const T &b) { return a <= b; } :
                                  [](const T &a, const T &b) { return a >= b; }) {}

    // 返回切片索引
    std::pair<int, int> operator()(T value) {
      ASSERT(mIndex < mValues.size(), "ValueSlicer: The slicer has expired")
      ASSERT(mCompare(mValues[mIndex], value), "ValueSlicer: Invalid input value")
      int i = mIndex;
      for (; mIndex < mValues.size(); ++mIndex) {
        if (!mCompare(mValues[mIndex], value)) break;
      }
      return {i, mIndex};
    }
};

}

#endif
