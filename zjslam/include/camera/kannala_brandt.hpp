#ifndef ZJSLAM__CAMERA__KANNALA_BRANDT_HPP
#define ZJSLAM__CAMERA__KANNALA_BRANDT_HPP

#include "base.hpp"
#include "pinhole.hpp"

// mvParam: [fx, fy, cx, cy, k0, k1, k2, k3]
#define KANNALA_BRANDT_NPARAM 8

// 3D -> 2D
#define KANNALA_BRANDT_PREPARE_BY_XYZ(p3D) \
  float R = this->computeR(atan2f(hypot(p3D.x, p3D.y), p3D.z)); \
  float psi = atan2f(p3D.y, p3D.x);

#define KANNALA_BRANDT_PREPARE_BY_VEC3(v3D) \
  float R = this->computeR(atan2f(hypot(v3D[0], v3D[1]), v3D[2])); \
  float psi = atan2f(v3D[1], v3D[0]);

#define KANNALA_BRANDT_PROJECT(vp) {vp[0] * R * cosf(psi) + vp[2], vp[1] * R * sinf(psi) + vp[3]}

// 2D -> 3D
#define KANNALA_BRANDT_UNPROJECT_PRECISION 1e-6


class KannalaBrandt8 : public CameraBase {
    CAMERA_DERIVED_SERIALIZE

    float computeR(float theta) const {
      float theta2 = theta * theta;
      return theta + theta2 * (mvParam[4] + theta2 * (mvParam[5] + theta2 * (mvParam[6] + theta2 * mvParam[7])));
    }

public:
    PINHOLE_FUNCTION_GETK

    KannalaBrandt8() { mvParam.resize(KANNALA_BRANDT_NPARAM); }

    KannalaBrandt8(const std::vector<float> &vParam) : CameraBase(vParam) { assert(mvParam.size() == KANNALA_BRANDT_NPARAM); }

    CameraType getType() const override { return CameraType::FISHEYE; }

    // 3D -> 2D
    cv::Point2f project(const cv::Point3f &p3D) const override {
      KANNALA_BRANDT_PREPARE_BY_XYZ(p3D)
      return KANNALA_BRANDT_PROJECT(mvParam);
    }

    Eigen::Vector2d project(const Eigen::Vector3d &v3D) const override {
      KANNALA_BRANDT_PREPARE_BY_VEC3(v3D)
      return KANNALA_BRANDT_PROJECT(mvParam);
    }

    Eigen::Vector2f project(const Eigen::Vector3f &v3D) const override {
      KANNALA_BRANDT_PREPARE_BY_VEC3(v3D)
      return KANNALA_BRANDT_PROJECT(mvParam);
    }

    Eigen::Vector2f projectEig(const cv::Point3f &p3D) const override {
      KANNALA_BRANDT_PREPARE_BY_XYZ(p3D)
      return KANNALA_BRANDT_PROJECT(mvParam);
    }

    // 2D -> 3D
    cv::Point3f unproject(const cv::Point2f &p2D) const override { return cv::Point3f(); }

    Eigen::Vector3f unprojectEig(const cv::Point2f &p2D) const override {
      // 世界坐标系
      float wx = (p2D.x - mvParam[2]) / mvParam[0];
      float wy = (p2D.y - mvParam[3]) / mvParam[1];
      float wz = 1.f;
      // R = poly(theta)
      float R = hypot(wx, wy);
      if (R > 1e-8) {
        float theta = R;
        // 最小化损失: (R - poly(theta))^2
        for (int i = 0; i < 10; i++) {
          float theta2 = theta * theta, theta4 = theta2 * theta2, theta6 = theta4 * theta2, theta8 = theta6 * theta2;
          float k0_theta2 = mvParam[4] * theta2, k1_theta4 = mvParam[5] * theta4,
              k2_theta6 = mvParam[6] * theta6, k3_theta8 = mvParam[7] * theta8;
          // 梯度下降法: g = 2 * (R - poly(theta)) * poly'(theta)
          float grad = (theta * (1 + k0_theta2 + k1_theta4 + k2_theta6 + k3_theta8) - R) /
                       (1 + 3 * k0_theta2 + 5 * k1_theta4 + 7 * k2_theta6 + 9 * k3_theta8);
          theta -= grad;
          if (abs(grad) < KANNALA_BRANDT_UNPROJECT_PRECISION) break;
        }
        wz = R / tanf(theta);
      }
      return {wx / wz, wy / wz, 1.f};
    }
};

#endif
