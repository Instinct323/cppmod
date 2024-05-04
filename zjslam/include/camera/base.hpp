#ifndef ZJSLAM__CAMERA__BASE_HPP
#define ZJSLAM__CAMERA__BASE_HPP

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/base_object.hpp>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

#include "../logging.hpp"

namespace camera {

typedef std::vector<float> Vectorf;


enum CameraType {
    PINHOLE, FISHEYE
};


class Base {

protected:
    cv::Size mImgSize;
    Vectorf mvParam;
    cv::Mat mMap1, mMap2;   // 畸变矫正映射

public:
    const Sophus::SE3d T_cam_imu;

    typedef std::shared_ptr<Base> Ptr;

    explicit Base(const cv::Size imgSize, const Vectorf &intrinsics, const Vectorf &distCoeffs,
                  const Sophus::SE3d &T_cam_imu = Sophus::SE3d()
    ) : mImgSize(imgSize), mvParam(intrinsics), T_cam_imu(T_cam_imu) {
      ASSERT(intrinsics.size() == 4, "Intrinsics size must be 4")
      mvParam.insert(mvParam.end(), distCoeffs.begin(), distCoeffs.end());
    }

    virtual CameraType getType() const = 0;

    // 参数读取
    inline float getParam(int i) const { return mvParam[i]; }

    inline size_t getParamSize() const { return mvParam.size(); }

    // 内参矩阵 K
#define GETK(vp, K) (K << vp[0], 0.f, vp[2], 0.f, vp[1], vp[3], 0.f, 0.f, 1.f)

    virtual cv::Mat getK() const { return GETK(mvParam, cv::Mat_<float>(3, 3)); };

    virtual Eigen::Matrix3f getKEig() const { return GETK(mvParam, Eigen::Matrix3f()).finished(); };

    // 3D -> 2D
    virtual cv::Point2f project(const cv::Point3f &p3D) const = 0;

    virtual Eigen::Vector2d project(const Eigen::Vector3d &v3D) const = 0;

    virtual Eigen::Vector2f project(const Eigen::Vector3f &v3D) const = 0;

    virtual Eigen::Vector2f projectEig(const cv::Point3f &p3D) const = 0;

    // 2D -> 3D
    virtual cv::Point3f unproject(const cv::Point2f &p2D) const = 0;

    virtual Eigen::Vector3f unprojectEig(const cv::Point2f &p2D) const = 0;

    // 去畸变
    virtual void undistort(const cv::Mat &src, cv::Mat &dst) = 0;

    // 运算符
    bool operator==(const Base &other) {
      if (this == &other) return true;
      if (this->getType() != other.getType() || this->getParamSize() != other.getParamSize()) return false;
      for (int i = 0; i < this->getParamSize(); i++) {
        if (this->getParam(i) != other.getParam(i)) return false;
      }
      return true;
    }
};
}

#endif
