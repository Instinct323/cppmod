#ifndef ZJCV__CAMERA__PINHOLE_HPP
#define ZJCV__CAMERA__PINHOLE_HPP

#include "base.hpp"

namespace camera {

// 3D -> 2D
#define PINHOLE_PROJECT(vp, x, y, z) \
  return {vp[0] * x / z + vp[2], vp[1] * y / z + vp[3]};

// 2D -> 3D
#define PINHOLE_UNPROJECT(vp, x, y) \
  return {(x - vp[2]) / vp[0], (y - vp[3]) / vp[1], 1.f};


class Pinhole : public Base {
    cv::Mat mOrgK, mRectR;

public:
    typedef std::shared_ptr<Pinhole> Ptr;

    using Base::undistort;

    explicit Pinhole(const cv::Size imgSize, const Vectorf &intrinsics, const Vectorf &distCoeffs,
                     const Sophus::SE3f &T_cam_imu = Sophus::SE3f()
    ) : Base(imgSize, intrinsics, distCoeffs, T_cam_imu), mOrgK(getK()) {
      ASSERT(distCoeffs.size() >= 4, "Distortion coefficients size must be at least 4")
      // 计算畸变矫正映射
      cv::initUndistortRectifyMap(mOrgK, distCoeffs, cv::Mat(), mOrgK, mImgSize, CV_32FC1, mMap1, mMap2);
    }

    Pinhole(const Pinhole &) = delete;

    CameraType get_type() const override { return CameraType::PINHOLE; }

    // 3D -> 2D
    cv::Point2f project(const Eigen::Vector3f &v3D) const override { PINHOLE_PROJECT(mvParam, v3D[0], v3D[1], v3D[2]) }

    Eigen::Vector2d project(const Eigen::Vector3d &v3D) const override { PINHOLE_PROJECT(mvParam, v3D[0], v3D[1], v3D[2]) }

    Eigen::Vector2f project_eig(const cv::Point3f &p3D) const override { PINHOLE_PROJECT(mvParam, p3D.x, p3D.y, p3D.z) }

    // 2D -> 3D
    Eigen::Vector3f unproject(const cv::Point2f &p2D) const override { PINHOLE_UNPROJECT(mvParam, p2D.x, p2D.y) }

    // 去畸变
    void undistort(const cv::Mat &src, cv::Mat &dst) const override { cv::remap(src, dst, mMap1, mMap2, cv::INTER_LINEAR); }

    void undistort(const VectorPt2f &src, VectorPt2f &dst) const override { cv::undistortPoints(src, dst, mOrgK, get_distcoeffs(), mRectR, getK()); }

    // 立体校正: 调用后, 由 undistot 函数驱动
    void stereo_rectify(Pinhole *cam_right);
};

}

#endif
