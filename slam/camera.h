#pragma once

#include <sophus/se3.hpp>

#include "utils.h"


/** @brief 相机 */
class Camera {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Camera> Ptr;

    double fx = 1, fy = 1, cx = 0, cy = 0;
    SE3 Tcr, Trc, Trw, Twr, Tcw, Twc;   // camera, robot, world

    // 返回内参
    Mat33 K() const {
      Mat33 k;
      k << fx, 0, cx, 0, fy, cy, 0, 0, 1;
      return k;
    }

    // 变换矩阵
    void set_Tcr(const SE3 &T) {
      Tcr = T;
      Trc = T.inverse();
      update_Tcw();
    }

    void set_Trw(const SE3 &T) {
      Trw = T;
      Twr = T.inverse();
      update_Tcw();
    }

    void update_Tcw() {
      Tcw = Tcr * Trw;
      Twc = Tcw.inverse();
    }

    // 坐标变换
    Vec2 camera2pixel(const Vec3 &p_c) const {
      return {fx * p_c(0) / p_c(2) + cx,
              fy * p_c(1) / p_c(2) + cy};
    }

    Vec3 pixel2camera(const Vec2 &p_p, double depth) const {
      return {(p_p(0) - cx) / fx * depth,
              (p_p(1) - cy) / fy * depth,
              depth};
    }

    Vec3 world2camera(const Vec3 &p_w) const { return Tcw * p_w; }

    Vec3 camera2world(const Vec3 &p_c) const { return Twc * p_c; }

    Vec2 world2pixel(const Vec3 &p_w) const { return camera2pixel(world2camera(p_w)); }

    Vec3 pixel2world(const Vec2 &p_p, double depth) const { return camera2world(pixel2camera(p_p, depth)); }
};
