#ifndef ZJSLAM__CAMERA__PINHOLE_HPP
#define ZJSLAM__CAMERA__PINHOLE_HPP

#include "base.hpp"

// 2D -> 3D
#define PINHOLE_PROJECT_BY_XYZ(vp, p3D) {vp[0] * p3D.x / p3D.z + vp[2], vp[1] * p3D.y / p3D.z + vp[3]}
#define PINHOLE_PROJECT_BY_VEC3(vp, v3D) {vp[0] * v3D[0] / v3D[2] + vp[2], vp[1] * v3D[1] / v3D[2] + vp[3]}
// 3D -> 2D
#define PINHOLE_PROJECT_BY_XY(vp, p2D) {(p2D.x - vp[2]) / vp[0], (p2D.y - vp[3]) / vp[1], 1.f}


class Pinhole : public CameraBase {
    // 序列化
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version) {
      ar & boost::serialization::base_object<CameraBase>(*this);
    }

protected:
    // mvParam: [fx, fy, cx, cy]
    static int nParam;

public:
    Pinhole() { mvParam.resize(nParam); }

    Pinhole(const std::vector<float> &vParam) : CameraBase(vParam) { assert(mvParam.size() == nParam); }

    CameraType getType() { return CameraType::PINHOLE; }

    // 2D -> 3D
    cv::Point2f project(const cv::Point3f &p3D) { return PINHOLE_PROJECT_BY_XYZ(mvParam, p3D); }

    Eigen::Vector2d project(const Eigen::Vector3d &v3D) { return PINHOLE_PROJECT_BY_VEC3(mvParam, v3D); }

    Eigen::Vector2f project(const Eigen::Vector3f &v3D) { return PINHOLE_PROJECT_BY_VEC3(mvParam, v3D); }

    Eigen::Vector2f projectEig(const cv::Point3f &p3D) { return PINHOLE_PROJECT_BY_XYZ(mvParam, p3D); }

    // 3D -> 2D
    cv::Point3f unproject(const cv::Point2f &p2D) { return PINHOLE_PROJECT_BY_XY(mvParam, p2D); }

    Eigen::Vector3f unprojectEig(const cv::Point2f &p2D) { return PINHOLE_PROJECT_BY_XY(mvParam, p2D); }
};


int Pinhole::nParam = 4;

#endif
