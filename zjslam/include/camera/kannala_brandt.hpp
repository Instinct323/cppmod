#ifndef ZJSLAM__CAMERA__KANNALA_BRANDT_HPP
#define ZJSLAM__CAMERA__KANNALA_BRANDT_HPP

#include "base.hpp"

namespace camera {


class KannalaBrandt8 : public Base {

public:
    explicit KannalaBrandt8(const cv::Size imgSize, const Vectorf &intrinsics, const Vectorf &distCoeffs
    ) : Base(imgSize, intrinsics, distCoeffs) {
      ASSERT(distCoeffs.size() == 4, "Distortion coefficients size must be 4")
    }

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

    // 去畸变
    void undistort(const cv::Mat &src, cv::Mat &dst) override {if (dst.empty()) dst = src.clone();}

    void undistortShow(const cv::Mat &src) {
      LOG(WARNING) << "A deprecated method is being called";
      if (mMap1.empty()) {
        mMap1 = cv::Mat(mImgSize, CV_32FC1), mMap2 = mMap1.clone();
        // 获取 3D 边界
        float W = mImgSize.width - 1, H = mImgSize.height - 1;
        float x = this->unproject({0, H / 2}).x, y = this->unproject({W / 2, 0}).y,
            w = this->unproject({W, H / 2}).x - x, h = this->unproject({W / 2, H}).y - y;
        // 计算畸变矫正映射
        for (int r = 0; r < H; ++r) {
          for (int c = 0; c < W; ++c) {
            cv::Point2f p2D = this->project(cv::Point3f(w * c / W + x, h * r / H + y, 1));
            mMap1.at<float>(r, c) = p2D.x;
            mMap2.at<float>(r, c) = p2D.y;
          }
        }
      }
      cv::Mat dst;
      cv::remap(src, dst, mMap1, mMap2, cv::INTER_LINEAR);
      cv::imshow("undistort", dst);
    }

protected:
    // 3D -> 2D: R(theta)
    float computeR(float theta) const {
      float theta2 = theta * theta;
      return theta + theta2 * (mvParam[4] + theta2 * (mvParam[5] + theta2 * (mvParam[6] + theta2 * mvParam[7])));
    }

    // 2D -> 3D: wz(wx, wy)
    float solveWZ(float wx, float wy, size_t iterations = 10) const {
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
};
}

#endif
