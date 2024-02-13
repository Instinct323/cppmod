#pragma once

#include <utility>

#include "mappoint.h"


class Keypoint {

public:
    Mappoint::Ptr mp = nullptr;
    float x, y;

    // 类型转换
    inline operator Vec2() { return {x, y}; }

    inline operator cv::Point2f() { return {x, y}; }

    // 构造函数
    explicit Keypoint(cv::Point2f &pt,
                      Mappoint::Ptr mp = Mappoint::Ptr()
    ) : x(pt.x), y(pt.y), mp(std::move(mp)) {}

    // 拷贝构造函数
    Keypoint(const cv::Point2f &other) : x(other.x), y(other.y) {}

    Keypoint(const Keypoint &other) = default;

    // 拷贝赋值运算符
    inline Keypoint &operator=(const Vec2 &other) {
      x = static_cast<float>(other[0]);
      y = static_cast<float>(other[1]);
      return *this;
    }

    inline Keypoint &operator=(const cv::Point2f &other) {
      x = other.x;
      y = other.y;
      return *this;
    }

    inline Keypoint &operator=(const Keypoint &other) {
      if (this != &other) {
        x = other.x;
        y = other.y;
        mp = other.mp;
      }
      return *this;
    }

protected:
    friend std::ostream &operator<<(std::ostream &os, const Keypoint &kp) {
      format fmt("Keypoint(%.2f, %.2f");
      os << (fmt % kp.x % kp.y);
      if (kp.mp != nullptr) { os << ", " << *kp.mp; }
      return os << ")";
    }
};
