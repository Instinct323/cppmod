#ifndef ZJSLAM__CAMERA__KANNALA_BRANDT_HPP
#define ZJSLAM__CAMERA__KANNALA_BRANDT_HPP

#include "base.hpp"
#include "pinhole.hpp"

// mvParam: [fx, fy, cx, cy, k0, k1, k2, k3]
#define KANNALA_BRANDT_NPARAM 8


class KannalaBrandt8 : public CameraBase {
    CAMERA_DERIVED_SERIALIZE

public:
    PINHOLE_FUNCTION_GETK

    KannalaBrandt8() { mvParam.resize(KANNALA_BRANDT_NPARAM); }

    KannalaBrandt8(const std::vector<float> &vParam) : CameraBase(vParam) { assert(mvParam.size() == KANNALA_BRANDT_NPARAM); }

    CameraType getType() const override { return CameraType::FISHEYE; }

// 3D -> 2D
#define KANNALA_BRANDT_PROJECT_BY_XYZ(vp, p3D) \
  float R = this->computeR(atan2f(hypot(p3D.x, p3D.y), p3D.z)); \
  float psi = atan2f(p3D.y, p3D.x); \
  return {vp[0] * R * cosf(psi) + vp[2], vp[1] * R * sinf(psi) + vp[3]};

#define KANNALA_BRANDT_PROJECT_BY_VEC3(vp, v3D) \
  float R = this->computeR(atan2f(hypot(v3D[0], v3D[1]), v3D[2])); \
  float psi = atan2f(v3D[1], v3D[0]); \
  return {vp[0] * R * cosf(psi) + vp[2], vp[1] * R * sinf(psi) + vp[3]};

    cv::Point2f project(const cv::Point3f &p3D) const override { KANNALA_BRANDT_PROJECT_BY_XYZ(mvParam, p3D) }

    Eigen::Vector2d project(const Eigen::Vector3d &v3D) const override { KANNALA_BRANDT_PROJECT_BY_VEC3(mvParam, v3D) }

    Eigen::Vector2f project(const Eigen::Vector3f &v3D) const override { KANNALA_BRANDT_PROJECT_BY_VEC3(mvParam, v3D) }

    Eigen::Vector2f projectEig(const cv::Point3f &p3D) const override { KANNALA_BRANDT_PROJECT_BY_XYZ(mvParam, p3D) }

// 2D -> 3D
#define KANNALA_BRANDT_UNPROJECT_PRECISION 1e-6

#define KANNALA_BRANDT_UNPROJECT_BY_XY(vp, p2D) \
  float wx = (p2D.x - vp[2]) / vp[0]; \
  float wy = (p2D.y - vp[3]) / vp[1]; \
  float wz = this->solveWZ(wx, wy); \
  return {wx / wz, wy / wz, 1.f};

    cv::Point3f unproject(const cv::Point2f &p2D) const override { KANNALA_BRANDT_UNPROJECT_BY_XY(mvParam, p2D) }

    Eigen::Vector3f unprojectEig(const cv::Point2f &p2D) const override { KANNALA_BRANDT_UNPROJECT_BY_XY(mvParam, p2D) }

private:
    // 3D -> 2D: R(theta)
    float computeR(float theta) const {
      float theta2 = theta * theta;
      return theta + theta2 * (mvParam[4] + theta2 * (mvParam[5] + theta2 * (mvParam[6] + theta2 * mvParam[7])));
    }

    // 2D -> 3D: wz(wx, wy)
    float solveWZ(float wx, float wy) const {
      // wz = lim_{theta -> 0} R / tan(theta) = 1
      float wz = 1.f;
      float R = hypot(wx, wy);
      float maxR = this->computeR(M_PI_2);
      // 超出最大半径
      if (R >= maxR) {
        wz = R / maxR;
      } else if (R > KANNALA_BRANDT_UNPROJECT_PRECISION) {
        float theta = M_PI_2;
        // 最小化损失: (poly(theta) - R)^2
        for (int i = 0; i < 10; i++) {
          float theta2 = theta * theta, theta4 = theta2 * theta2, theta6 = theta4 * theta2, theta8 = theta6 * theta2;
          float k0_theta2 = mvParam[4] * theta2, k1_theta4 = mvParam[5] * theta4,
              k2_theta6 = mvParam[6] * theta6, k3_theta8 = mvParam[7] * theta8;
          float curR = theta * (1 + k0_theta2 + k1_theta4 + k2_theta6 + k3_theta8);
          float e = curR - R;
          if (abs(e) < KANNALA_BRANDT_UNPROJECT_PRECISION) break;
          // 梯度下降法: g = (poly(theta) - R) / poly'(theta)
          float grad = e / (1 + 3 * k0_theta2 + 5 * k1_theta4 + 7 * k2_theta6 + 9 * k3_theta8);
          theta -= grad;
        }
        wz = R / tanf(theta);
      }
      return wz;
    }
};

#endif