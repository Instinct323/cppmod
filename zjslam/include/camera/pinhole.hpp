#ifndef ZJSLAM__CAMERA__PINHOLE_HPP
#define ZJSLAM__CAMERA__PINHOLE_HPP

#include "base.hpp"

namespace camera {

// mvParam: [fx, fy, cx, cy]
#define PINHOLE_NPARAM 4

// 内参矩阵 K
#define PINHOLE_GETK(vp, K) (K << vp[0], 0.f, vp[2], 0.f, vp[1], vp[3], 0.f, 0.f, 1.f)

#define PINHOLE_FUNCTION_GETK \
  cv::Mat getK() const override { return PINHOLE_GETK(mvParam, cv::Mat_<float>(3, 3)); } \
  Eigen::Matrix3f getKEig() const override { return PINHOLE_GETK(mvParam, Eigen::Matrix3f()).finished(); }


    class Pinhole : public Base {
        CAMERA_DERIVED_SERIALIZE

    public:
        PINHOLE_FUNCTION_GETK

        Pinhole() { mvParam.resize(PINHOLE_NPARAM); }

        Pinhole(const std::vector<float> &vParam) : Base(vParam) { assert(mvParam.size() == PINHOLE_NPARAM); }

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
    };
}

#endif
