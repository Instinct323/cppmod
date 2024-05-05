#ifndef ZJSLAM__CAMERA__BASE_HPP
#define ZJSLAM__CAMERA__BASE_HPP

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
    Sophus::SE3d T_cam_imu;

    typedef std::shared_ptr<Base> Ptr;

    explicit Base(const cv::Size imgSize, const Vectorf &intrinsics, const Vectorf &distCoeffs,
                  const Sophus::SE3d &T_cam_imu = Sophus::SE3d()
    ) : mImgSize(imgSize), mvParam(intrinsics), T_cam_imu(T_cam_imu) {
      ASSERT(intrinsics.size() == 4, "Intrinsics size must be 4")
      mvParam.insert(mvParam.end(), distCoeffs.begin(), distCoeffs.end());
    }

    virtual CameraType getType() const = 0;

    // 参数读取
    inline void setParam(int i, float value) { mvParam[i] = value; }

    inline float getParam(int i) const { return mvParam[i]; }

    inline size_t getParamSize() const { return mvParam.size(); }

    Vectorf getDistCoeffs() const { return {mvParam.begin() + 4, mvParam.end()}; }

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

    // 绘制归一化平面 (z=1)
    void drawNormalizedPlane(const cv::Mat &src, cv::Mat &dst);
};


void Base::drawNormalizedPlane(const cv::Mat &src, cv::Mat &dst) {
  undistort(src, dst);
  cv::Mat npMap1 = cv::Mat(mImgSize, CV_32FC1), npMap2 = npMap1.clone();
  // 获取归一化平面边界 (桶形畸变)
  float x, y, w, h, W = mImgSize.width - 1, H = mImgSize.height - 1;
  x = this->unproject({0, H / 2}).x, y = this->unproject({W / 2, 0}).y,
      w = this->unproject({W, H / 2}).x - x, h = this->unproject({W / 2, H}).y - y;
  LOG(INFO) << "Normalized plane: " << cv::Vec4f(x, y, x + w, y + h);
  // 计算畸变矫正映射
  for (int r = 0; r < H; ++r) {
    for (int c = 0; c < W; ++c) {
      cv::Point2f p2D = this->project(cv::Point3f(w * c / W + x, h * r / H + y, 1));
      npMap1.at<float>(r, c) = p2D.x;
      npMap2.at<float>(r, c) = p2D.y;
    }
  }
  cv::remap(dst, dst, npMap1, npMap2, cv::INTER_LINEAR);
}
}

#endif
