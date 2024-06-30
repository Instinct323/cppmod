#ifndef UTILS__MATH_HPP
#define UTILS__MATH_HPP

#include <Eigen/Core>

#include "glog.hpp"

namespace math {


// std::vector -> Eigen
template<typename T>
Eigen::Vector<T, -1> toEigen(const std::vector<T> &vec) {
  Eigen::Vector<T, -1> v(vec.size());
  for (int i = 0; i < vec.size(); i++) v(i) = vec[i];
  return v;
}


// mean
template<typename T>
double mean(const std::vector<T> &vec) {
  double sum = 0;
  for (const T &value: vec) sum += static_cast<double>(value);
  return sum / vec.size();
}


// standard deviation
template<typename T>
double std(const std::vector<T> &vec, double vMean) {
  double var = 0;
  for (const T &value: vec) var += pow(static_cast<double>(value) - vMean, 2);
  return sqrt(var / vec.size());
}


template<typename T>
double std(const std::vector<T> &vec) { return std(vec, mean(vec)); }


// 拉依达准则
template<typename T>
class PautaCriterion {
    double mMean, mScaledStd;

public:
    explicit PautaCriterion(const std::vector<T> &vec, double sigmaFactor = 3
    ) : mMean(mean(vec)), mScaledStd(sigmaFactor * std(vec, mMean)) {}

    bool operator()(const T &value) { return abs(static_cast<double>(value) - mMean) < mScaledStd; }
};


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
