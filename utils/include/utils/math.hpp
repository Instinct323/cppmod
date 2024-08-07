#ifndef UTILS__MATH_HPP
#define UTILS__MATH_HPP

#include <Eigen/Core>

#include "glog.hpp"

namespace math {

// clamp
template<typename T>
T clamp(T value, T amin, T amax) { return value < amin ? amin : (value > amax ? amax : value); }


// std::vector -> Eigen
template<typename T>
Eigen::Vector<T, -1> toEigen(const std::vector<T> &vec) {
  Eigen::Vector<T, -1> v(vec.size());
  for (int i = 0; i < vec.size(); i++) v(i) = vec[i];
  return v;
}


// mean
template<typename T>
float mean(const std::vector<T> &vec) {
  long double sum = 0;
  for (const T &value: vec) sum += value;
  return sum / vec.size();
}


// mean square
template<typename T>
float mean_square(const std::vector<T> &vec) {
  long double sum = 0;
  for (const T &value: vec) sum += value * value;
  return sum / vec.size();
}


// standard deviation
template<typename T>
float std(const std::vector<T> &vec, float vMean) {
  long double var = 0;
  for (const T &value: vec) var += pow(static_cast<float>(value) - vMean, 2);
  return sqrt(var / vec.size());
}


template<typename T>
float std(const std::vector<T> &vec) { return std(vec, mean(vec)); }


// 指数移动平均
class EMA {
    float mMotion;
    float *mValue;

public:
    explicit EMA(float motion = 0.1) : mMotion(motion) {}

    ~EMA() { delete mValue; }

    // 值更新
    template<typename T>
    void update(T value) {
      static_assert(std::is_arithmetic<T>::value, "EMA: Invalid type");
      if (mValue == nullptr) {
        mValue = new float(value);
      } else {
        *mValue = mMotion * value + (1 - mMotion) * (*mValue);
      }
    }

    // 值读取
    float get() { return *mValue; }

    template<typename T>
    explicit operator T() {
      static_assert(std::is_floating_point<T>::value, "EMA: Invalid type");
      return *mValue;
    }
};


// 拉依达准则
template<typename T>
class PautaCriterion {

public:
    float mMean, mScaledStd;

    explicit PautaCriterion(const std::vector<T> &vec, float sigma_factor = 3
    ) : mMean(mean(vec)), mScaledStd(sigma_factor * std(vec, mMean)) {}

    bool operator()(const T &value) { return abs(static_cast<float>(value) - mMean) < mScaledStd; }
};


// 按值切片器
template<typename T>
class ValueSlicer {
    int mIndex = 0;
    const std::vector<T> *mpValues;
    std::function<bool(const T &, const T &)> mCompare;

public:
    explicit ValueSlicer(const std::vector<T> *values, bool ascending = true
    ) : mpValues(values), mCompare(ascending ? [](const T &a, const T &b) { return a <= b; } :
                                   [](const T &a, const T &b) { return a >= b; }) {}

    // 返回切片索引
    std::pair<int, int> operator()(T value) {
      ASSERT(mIndex < mpValues->size(), "ValueSlicer: The slicer has expired")
      int i = mIndex;
      for (; mIndex < mpValues->size(); ++mIndex) {
        if (!mCompare(mpValues->operator[](mIndex), value)) break;
      }
      return {i, mIndex};
    }
};

}

#endif
