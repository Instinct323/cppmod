#ifndef ZJSLAM__CAMERA__BASE_HPP
#define ZJSLAM__CAMERA__BASE_HPP

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/base_object.hpp>
#include <cassert>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <vector>


enum CameraType {
    PINHOLE, FISHEYE
};


class CameraBase {
    // 序列化
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version) {
      ar & mvParam;
    }

protected:
    std::vector<float> mvParam;

public:
    CameraBase() = default;

    CameraBase(const std::vector<float> &vParam) : mvParam(vParam) {}

    virtual CameraType getType() = 0;

    // 参数读写
    inline void setParam(int i, float v) { mvParam[i] = v; }

    inline float getParam(int i) { return mvParam[i]; }

    inline size_t getParamSize() { return mvParam.size(); }

    // 2D -> 3D
    virtual cv::Point2f project(const cv::Point3f &p3D) = 0;

    virtual Eigen::Vector2d project(const Eigen::Vector3d &v3D) = 0;

    virtual Eigen::Vector2f project(const Eigen::Vector3f &v3D) = 0;

    virtual Eigen::Vector2f projectEig(const cv::Point3f &p3D) = 0;

    // 3D -> 2D
    virtual cv::Point3f unproject(const cv::Point2f &p2D) = 0;

    virtual Eigen::Vector3f unprojectEig(const cv::Point2f &p2D) = 0;
};

#endif
