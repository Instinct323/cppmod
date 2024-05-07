#ifndef ZJSLAM__CAMERA__PINHOLE_HPP
#define ZJSLAM__CAMERA__PINHOLE_HPP

#include "base.hpp"
#include "../convert.hpp"

namespace camera {

// 3D -> 2D
#define PINHOLE_PROJECT_BY_XYZ(vp, p3D) \
  return {vp[0] * p3D.x / p3D.z + vp[2], vp[1] * p3D.y / p3D.z + vp[3]};

#define PINHOLE_PROJECT_BY_VEC3(vp, v3D) \
  return {vp[0] * v3D[0] / v3D[2] + vp[2], vp[1] * v3D[1] / v3D[2] + vp[3]};

// 2D -> 3D
#define PINHOLE_UNPROJECT_BY_XY(vp, p2D) \
  return {(p2D.x - vp[2]) / vp[0], (p2D.y - vp[3]) / vp[1], 1.f};


class Pinhole : public Base {
    cv::Mat mOrgK, mRectR;

public:
    typedef std::shared_ptr<Pinhole> Ptr;

    explicit Pinhole(const cv::Size imgSize, const Vectorf &intrinsics, const Vectorf &distCoeffs,
                     const Sophus::SE3d &T_cam_imu = Sophus::SE3d()
    ) : Base(imgSize, intrinsics, distCoeffs, T_cam_imu), mOrgK(getK()) {
      ASSERT(distCoeffs.size() >= 4, "Distortion coefficients size must be at least 4")
      // 计算畸变矫正映射
      cv::initUndistortRectifyMap(mOrgK, distCoeffs, cv::Mat(), mOrgK, mImgSize, CV_32FC1, mMap1, mMap2);
    }

    CameraType getType() const override { return CameraType::PINHOLE; }

    // 3D -> 2D
    cv::Point2f project(const cv::Point3f &p3D) const override { PINHOLE_PROJECT_BY_XYZ(mvParam, p3D) }

    Eigen::Vector2d project(const Eigen::Vector3d &v3D) const override { PINHOLE_PROJECT_BY_VEC3(mvParam, v3D) }

    Eigen::Vector2f project(const Eigen::Vector3f &v3D) const override { PINHOLE_PROJECT_BY_VEC3(mvParam, v3D) }

    Eigen::Vector2f projectEig(const cv::Point3f &p3D) const override { PINHOLE_PROJECT_BY_XYZ(mvParam, p3D) }

    // 2D -> 3D
    cv::Point3f unproject(const cv::Point2f &p2D) const override { PINHOLE_UNPROJECT_BY_XY(mvParam, p2D) }

    Eigen::Vector3f unprojectEig(const cv::Point2f &p2D) const override { PINHOLE_UNPROJECT_BY_XY(mvParam, p2D) }

    // 去畸变
    void undistort(const cv::Mat &src, cv::Mat &dst) override { cv::remap(src, dst, mMap1, mMap2, cv::INTER_LINEAR); }

    void undistort(const VectorPt2f &src, VectorPt2f &dst) override {cv::undistortPoints(src, dst, mOrgK, getDistCoeffs(), mRectR, getK());}

    // 立体校正
    void stereoRectify(Pinhole *cam_right);
};


void Pinhole::stereoRectify(Pinhole *cam_right) {
  ASSERT(this->mImgSize == cam_right->mImgSize, "Image size must be the same")
  Sophus::SE3d Trl = this->T_cam_imu.inverse() * cam_right->T_cam_imu;
  cv::Mat P1, P2;
  cv::stereoRectify(this->getK(), this->getDistCoeffs(), cam_right->getK(), cam_right->getDistCoeffs(), mImgSize,
                    cvt::toCvMat<double>(Trl.rotationMatrix()),
                    cvt::toCvMat<double>(Trl.translation()), this->mRectR, cam_right->mRectR, P1, P2, cv::Mat());
  // 重新初始化畸变矫正映射
  cv::initUndistortRectifyMap(this->getK(), this->getDistCoeffs(), this->mRectR, P1, mImgSize, CV_32FC1, mMap1, mMap2);
  cv::initUndistortRectifyMap(cam_right->getK(), cam_right->getDistCoeffs(), cam_right->mRectR, P2, mImgSize, CV_32FC1,
                              cam_right->mMap1, cam_right->mMap2);
  // 原地修改相机内参
  int paramPos[2][4] = {{0, 1, 0, 1},
                        {0, 1, 2, 2}};
  for (int i = 0; i < 4; i++) {
    this->setParam(i, P1.at<double>(paramPos[0][i], paramPos[1][i]), true);
    cam_right->setParam(i, P2.at<double>(paramPos[0][i], paramPos[1][i]), true);
  }
  // 原地修改相机位姿
  Sophus::SE3d R1(cvt::toEigen<double>(this->mRectR), Eigen::Vector3d::Zero()),
      R2(cvt::toEigen<double>(cam_right->mRectR), Eigen::Vector3d::Zero());
  this->T_cam_imu = R1 * this->T_cam_imu;
  cam_right->T_cam_imu = R2 * cam_right->T_cam_imu;
}

}

#endif
