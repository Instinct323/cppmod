#ifndef ZJSLAM__CAMERA__BASE_HPP
#define ZJSLAM__CAMERA__BASE_HPP

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/base_object.hpp>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

namespace camera {

// Base 派生类序列化
#define CAMERA_DERIVED_SERIALIZE \
  friend class boost::serialization::access; \
  template<class Archive> \
    void serialize(Archive &ar, const unsigned int version) { \
    ar & boost::serialization::base_object<Base>(*this); }


enum CameraType {
    PINHOLE, FISHEYE
};


class Base {
    // 序列化
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version) {
      ar & mvParam;
    }

protected:
    std::vector<float> mvParam;

public:
    Base() = default;

    explicit Base(const std::vector<float> &vParam) : mvParam(vParam) {}

    virtual CameraType getType() const = 0;

    // 参数读写
    inline void setParam(int i, float v) { mvParam[i] = v; }

    inline float getParam(int i) const { return mvParam[i]; }

    inline size_t getParamSize() const { return mvParam.size(); }

    // 内参矩阵 K
    virtual cv::Mat getK() const = 0;

    virtual Eigen::Matrix3f getKEig() const = 0;

    // 3D -> 2D
    virtual cv::Point2f project(const cv::Point3f &p3D) const = 0;

    virtual Eigen::Vector2d project(const Eigen::Vector3d &v3D) const = 0;

    virtual Eigen::Vector2f project(const Eigen::Vector3f &v3D) const = 0;

    virtual Eigen::Vector2f projectEig(const cv::Point3f &p3D) const = 0;

    // 2D -> 3D
    virtual cv::Point3f unproject(const cv::Point2f &p2D) const = 0;

    virtual Eigen::Vector3f unprojectEig(const cv::Point2f &p2D) const = 0;

    // 运算符
    bool operator==(const Base &other) {
      if (this == &other) return true;
      if (getType() != other.getType() || getParamSize() != other.getParamSize()) return false;
      for (int i = 0; i < getParamSize(); i++) {
        if (getParam(i) != other.getParam(i)) return false;
      }
      return true;
    }

    friend std::ostream &operator<<(std::ostream &os, const Base &camera) {
      os << "Camera-" << camera.getType() << "(";
      for (int i = 0; i < camera.getParamSize(); i++) os << camera.getParam(i) << ", ";
      return os << ")";
    }
};
}

#endif
