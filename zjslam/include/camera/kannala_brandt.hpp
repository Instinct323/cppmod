#ifndef ZJSLAM__CAMERA__KANNALA_BRANDT_HPP
#define ZJSLAM__CAMERA__KANNALA_BRANDT_HPP

#include "base.hpp"

namespace camera {

// 3D -> 2D
#define KANNALA_BRANDT_PROJECT_BY_XYZ(vp, p3D) \
  float R = this->computeR(atan2f(hypot(p3D.x, p3D.y), p3D.z)); \
  float psi = atan2f(p3D.y, p3D.x); \
  return {vp[0] * R * cosf(psi) + vp[2], vp[1] * R * sinf(psi) + vp[3]};

#define KANNALA_BRANDT_PROJECT_BY_VEC3(vp, v3D) \
  float R = this->computeR(atan2f(hypot(v3D[0], v3D[1]), v3D[2])); \
  float psi = atan2f(v3D[1], v3D[0]); \
  return {vp[0] * R * cosf(psi) + vp[2], vp[1] * R * sinf(psi) + vp[3]};

// 2D -> 3D
#define KANNALA_BRANDT_UNPROJECT_PRECISION 1e-6

#define KANNALA_BRANDT_UNPROJECT_BY_XY(p2D) \
  cv::Vec2f wxy = mUnprojectCache.at<cv::Vec2f>(p2D.y, p2D.x); \
  return {wxy[0], wxy[1], 1};


class KannalaBrandt8 : public Base {

protected:
    cv::Mat mUnprojectCache;

    void makeUnprojectCache();

public:
    typedef std::shared_ptr<KannalaBrandt8> Ptr;

    explicit KannalaBrandt8(const cv::Size imgSize, const Vectorf &intrinsics, const Vectorf &distCoeffs,
                            const Sophus::SE3d &T_cam_imu = Sophus::SE3d()
    ) : Base(imgSize, intrinsics, distCoeffs, T_cam_imu), mUnprojectCache(mImgSize, CV_32FC2) {
      ASSERT(distCoeffs.size() == 4, "Distortion coefficients size must be 4")
      makeUnprojectCache();
    }

    CameraType getType() const override { return CameraType::FISHEYE; }

    // 3D -> 2D
    float computeR(float theta) const;

    cv::Point2f project(const cv::Point3f &p3D) const override { KANNALA_BRANDT_PROJECT_BY_XYZ(mvParam, p3D) }

    Eigen::Vector2d project(const Eigen::Vector3d &v3D) const override { KANNALA_BRANDT_PROJECT_BY_VEC3(mvParam, v3D) }

    Eigen::Vector2f project(const Eigen::Vector3f &v3D) const override { KANNALA_BRANDT_PROJECT_BY_VEC3(mvParam, v3D) }

    Eigen::Vector2f projectEig(const cv::Point3f &p3D) const override { KANNALA_BRANDT_PROJECT_BY_XYZ(mvParam, p3D) }

    // 2D -> 3D
    float solveWZ(float wx, float wy, size_t iterations = 10) const;

    cv::Point3f unproject(const cv::Point2f &p2D) const override { KANNALA_BRANDT_UNPROJECT_BY_XY(p2D) }

    Eigen::Vector3f unprojectEig(const cv::Point2f &p2D) const override { KANNALA_BRANDT_UNPROJECT_BY_XY(p2D) }

    // 去畸变
    void undistort(const cv::Mat &src, cv::Mat &dst) override { if (dst.empty()) dst = src.clone(); }
};


float KannalaBrandt8::computeR(float theta) const {
  float theta2 = theta * theta;
  return theta + theta2 * (mvParam[4] + theta2 * (mvParam[5] + theta2 * (mvParam[6] + theta2 * mvParam[7])));
}


void KannalaBrandt8::makeUnprojectCache() {
  float wx, wy, wz;
  for (int r = 0; r < mImgSize.height; ++r) {
    wy = (r - mvParam[3]) / mvParam[1];
    for (int c = 0; c < mImgSize.width; ++c) {
      wx = (c - mvParam[2]) / mvParam[0];
      wz = this->solveWZ(wx, wy);
      mUnprojectCache.at<cv::Vec2f>(r, c) = {wx / wz, wy / wz};
    }
  }
}


float KannalaBrandt8::solveWZ(float wx, float wy, size_t iterations) const {
  // wz = lim_{theta -> 0} R / tan(theta) = 1
  float wz = 1.f;
  float R = hypot(wx, wy);
  float maxR = this->computeR(M_PI_2);
  // 超出 R 上限
  if (R >= maxR) {
    wz = R / maxR;
  } else if (R > KANNALA_BRANDT_UNPROJECT_PRECISION) {
    float theta = M_PI_2;
    // 最小化损失: (poly(theta) - R)^2
    int i = 0;
    float e;
    for (; i < iterations; i++) {
      float theta2 = theta * theta, theta4 = theta2 * theta2, theta6 = theta4 * theta2, theta8 = theta6 * theta2;
      float k0_theta2 = mvParam[4] * theta2, k1_theta4 = mvParam[5] * theta4,
          k2_theta6 = mvParam[6] * theta6, k3_theta8 = mvParam[7] * theta8;
      e = theta * (1 + k0_theta2 + k1_theta4 + k2_theta6 + k3_theta8) - R;
      if (abs(e) < R * KANNALA_BRANDT_UNPROJECT_PRECISION) break;
      // 梯度下降法: g = (poly(theta) - R) / poly'(theta)
      theta -= e / (1 + 3 * k0_theta2 + 5 * k1_theta4 + 7 * k2_theta6 + 9 * k3_theta8);
    }
    wz = R / tanf(theta);
    if (i == iterations) LOG(WARNING) << "solveWZ(" << wx << ", " << wy << "): relative error " << abs(e) / R;
  }
  return wz;
}
}

#endif
