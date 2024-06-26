#ifndef ZJCV__CAMERA__KANNALA_BRANDT_HPP
#define ZJCV__CAMERA__KANNALA_BRANDT_HPP

#include "base.hpp"

namespace camera {

// 最大视场角 (90)
#define KANNALA_BRANDT_MAX_FOV M_PI_2

// 3D -> 2D
#define KANNALA_BRANDT_PROJECT(vp, x, y, z) \
  float R = this->computeR(atan2f(hypot(x, y), z)); \
  float psi = atan2f(y, x); \
  return {vp[0] * R * cosf(psi) + vp[2], vp[1] * R * sinf(psi) + vp[3]};

// 2D -> 3D
#define KANNALA_BRANDT_UNPROJECT_PRECISION 1e-6

#define KANNALA_BRANDT_UNPROJECT(cache, x, y) \
  cv::Vec2f wxy = cache.at<cv::Vec2f>(y, x); \
  return {wxy[0], wxy[1], 1};


class KannalaBrandt : public Base {

protected:
    cv::Mat mUnprojectCache;

    void make_unproject_cache();

public:
    typedef std::shared_ptr<KannalaBrandt> Ptr;

    using Base::undistort;

    explicit KannalaBrandt(const cv::Size imgSize, const Vectorf &intrinsics, const Vectorf &distCoeffs,
                           const Sophus::SE3d &T_cam_imu = Sophus::SE3d()
    ) : Base(imgSize, intrinsics, distCoeffs, T_cam_imu), mUnprojectCache(mImgSize, CV_32FC2) {
      ASSERT(distCoeffs.size() == 4, "Distortion coefficients size must be 4")
      make_unproject_cache();
    }

    CameraType get_type() const override { return CameraType::KANNALA_BRANDT; }

    // 3D -> 2D
    float computeR(float theta) const;

    cv::Point2f project(const cv::Point3f &p3D) const override { KANNALA_BRANDT_PROJECT(mvParam, p3D.x, p3D.y, p3D.z) }

    Eigen::Vector2d project(const Eigen::Vector3d &v3D) const override { KANNALA_BRANDT_PROJECT(mvParam, v3D[0], v3D[1], v3D[2]) }

    Eigen::Vector2f project(const Eigen::Vector3f &v3D) const override { KANNALA_BRANDT_PROJECT(mvParam, v3D[0], v3D[1], v3D[2]) }

    Eigen::Vector2f project_eig(const cv::Point3f &p3D) const override { KANNALA_BRANDT_PROJECT(mvParam, p3D.x, p3D.y, p3D.z) }

    // 2D -> 3D
    float solveWZ(float wx, float wy, size_t iterations = 10) const;

    cv::Point3f unproject(const cv::Point2f &p2D) const override { KANNALA_BRANDT_UNPROJECT(mUnprojectCache, p2D.x, p2D.y) }

    Eigen::Vector3f unproject_eig(const cv::Point2f &p2D) const override { KANNALA_BRANDT_UNPROJECT(mUnprojectCache, p2D.x, p2D.y) }

    // 去畸变
    void undistort(const cv::Mat &src, cv::Mat &dst) override { if (src.data != dst.data) dst = src.clone(); }

    void undistort(const VectorPt2f &src, VectorPt2f &dst) override { if (src.data() != dst.data()) dst = src; }

    void undistort(const VectorKp &src, VectorKp &dst) override { if (src.data() != dst.data()) dst = src; }

    // ORB 特征
    void stereoORBfeatures(Base *pCamRight,
                           ORB::Extractor *pExtractor0, ORB::Extractor *pExtractor1,
                           const cv::Mat &img0, const cv::Mat &img1,
                           ORB::KeyPoints &kps0, ORB::KeyPoints &kps1,
                           cv::Mat &desc0, cv::Mat &desc1, std::vector<cv::DMatch> &matches) override;
};

}

#endif
