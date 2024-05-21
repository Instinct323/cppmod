#ifndef ZJSLAM__UTILS_HPP
#define ZJSLAM__UTILS_HPP

#include <boost/thread.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "logging.hpp"


// 线程共享变量
template<typename T>
struct SharedVar {
    boost::mutex mMutex;
    T mValue;

    SharedVar(T value) : mValue(value) {};
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
      ASSERT(mIndex < mValues.size(), "ValueSlicer: The slicer has expired");
      ASSERT(mCompare(mValues[mIndex], value), "ValueSlicer: Invalid input value");
      int i = mIndex;
      for (; mIndex < mValues.size(); ++mIndex) {
        if (!mCompare(mValues[mIndex], value)) break;
      }
      return {i, mIndex};
    }
};

#endif
