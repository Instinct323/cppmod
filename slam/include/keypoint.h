#pragma once

#include <utility>

#include "mappoint.h"


class Keypoint {

public:
    std::weak_ptr<Mappoint> _mappoint;
    float x, y;

    // 关联路标点
    bool hasMappoint() const { return !_mappoint.expired(); }

    Mappoint::Ptr getMappoint() const { return _mappoint.lock(); }

    // 类型转换
    operator Vec2() { return {x, y}; }

    operator cv::Point2f() { return {x, y}; }

    // 构造函数
    explicit Keypoint(cv::Point2f &pt,
                      Mappoint::Ptr mp = Mappoint::Ptr()
    ) : x(pt.x), y(pt.y), _mappoint(mp) {}

    // 拷贝构造函数
    Keypoint(const cv::Point2f &other) : x(other.x), y(other.y) {}

    Keypoint(const Keypoint &other) = default;

    // 拷贝赋值运算符
    Keypoint &operator=(const Vec2 &other) {
      x = other[0];
      y = other[1];
      return *this;
    }

    Keypoint &operator=(const cv::Point2f &other) {
      x = other.x;
      y = other.y;
      return *this;
    }

    Keypoint &operator=(const Keypoint &other) {
      if (this != &other) {
        x = other.x;
        y = other.y;
        _mappoint = other._mappoint;
      }
      return *this;
    }
};
