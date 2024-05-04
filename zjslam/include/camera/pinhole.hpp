#ifndef ZJSLAM__CAMERA__PINHOLE_HPP
#define ZJSLAM__CAMERA__PINHOLE_HPP

#include "base.hpp"

namespace camera {


class Pinhole : public Base {

public:
    typedef std::shared_ptr<Pinhole> Ptr;

    explicit Pinhole(const cv::Size imgSize, const Vectorf &intrinsics, const Vectorf &distCoeffs,
                     const Sophus::SE3d &T_cam_imu = Sophus::SE3d()
    ) : Base(imgSize, intrinsics, distCoeffs, T_cam_imu) {
      ASSERT(distCoeffs.size() >= 4, "Distortion coefficients size must be at least 4")
      // 计算畸变矫正映射
      cv::Mat K = getK();
      cv::initUndistortRectifyMap(K, distCoeffs, cv::Mat(), K, mImgSize, CV_32FC1, mMap1, mMap2);
    }

    CameraType getType() const override { return CameraType::PINHOLE; }

// 3D -> 2D
#define PINHOLE_PROJECT_BY_XYZ(vp, p3D) \
  return {vp[0] * p3D.x / p3D.z + vp[2], vp[1] * p3D.y / p3D.z + vp[3]};

#define PINHOLE_PROJECT_BY_VEC3(vp, v3D) \
  return {vp[0] * v3D[0] / v3D[2] + vp[2], vp[1] * v3D[1] / v3D[2] + vp[3]};

    cv::Point2f project(const cv::Point3f &p3D) const override { PINHOLE_PROJECT_BY_XYZ(mvParam, p3D) }

    Eigen::Vector2d project(const Eigen::Vector3d &v3D) const override { PINHOLE_PROJECT_BY_VEC3(mvParam, v3D) }

    Eigen::Vector2f project(const Eigen::Vector3f &v3D) const override { PINHOLE_PROJECT_BY_VEC3(mvParam, v3D) }

    Eigen::Vector2f projectEig(const cv::Point3f &p3D) const override { PINHOLE_PROJECT_BY_XYZ(mvParam, p3D) }

// 2D -> 3D
#define PINHOLE_UNPROJECT_BY_XY(vp, p2D) \
  return {(p2D.x - vp[2]) / vp[0], (p2D.y - vp[3]) / vp[1], 1.f};

    cv::Point3f unproject(const cv::Point2f &p2D) const override { PINHOLE_UNPROJECT_BY_XY(mvParam, p2D) }

    Eigen::Vector3f unprojectEig(const cv::Point2f &p2D) const override { PINHOLE_UNPROJECT_BY_XY(mvParam, p2D) }

    // 去畸变
    void undistort(const cv::Mat &src, cv::Mat &dst) override { cv::remap(src, dst, mMap1, mMap2, cv::INTER_LINEAR); }
};
}

#endif
