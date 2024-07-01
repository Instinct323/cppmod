#ifndef ZJCV__CAMERA__BASE_HPP
#define ZJCV__CAMERA__BASE_HPP

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

#include "utils/glog.hpp"
#include "utils/orb.hpp"

namespace camera {

typedef std::vector<float> Vectorf;
typedef std::vector<cv::Point2f> VectorPt2f;
typedef std::vector<cv::KeyPoint> VectorKp;


enum CameraType {
    PINHOLE, KANNALA_BRANDT
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

    virtual CameraType get_type() const = 0;

    // 参数读写
    inline void set_param(int i, float value, bool safe = false) {
      if (!safe) LOG(WARNING) << "Unsafe set_param: " << i << " = " << value;
      mvParam[i] = value;
    }

    inline float get_param(int i) const { return mvParam[i]; }

    inline size_t get_param_size() const { return mvParam.size(); }

    Vectorf get_distcoeffs() const { return {mvParam.begin() + 4, mvParam.end()}; }

    // 内参矩阵 K
#define GETK(vp, K) (K << vp[0], 0.f, vp[2], 0.f, vp[1], vp[3], 0.f, 0.f, 1.f)

    virtual cv::Mat getK() const { return GETK(mvParam, cv::Mat_<float>(3, 3)); };

    virtual Eigen::Matrix3f getK_eig() const { return GETK(mvParam, Eigen::Matrix3f()).finished(); };

    // 3D -> 2D
    virtual cv::Point2f project(const cv::Point3f &p3D) const = 0;

    virtual Eigen::Vector2d project(const Eigen::Vector3d &v3D) const = 0;

    virtual Eigen::Vector2f project(const Eigen::Vector3f &v3D) const = 0;

    virtual Eigen::Vector2f project_eig(const cv::Point3f &p3D) const = 0;

    // 2D -> 3D
    virtual cv::Point3f unproject(const cv::Point2f &p2D) const = 0;

    virtual Eigen::Vector3f unproject_eig(const cv::Point2f &p2D) const = 0;

    // 去畸变
    virtual void undistort(const cv::Mat &src, cv::Mat &dst) = 0;

    virtual void undistort(const VectorPt2f &src, VectorPt2f &dst) = 0;

    virtual void undistort(const VectorKp &src, VectorKp &dst) {
      VectorPt2f pts;
      for (const auto &kp: src) pts.push_back(kp.pt);
      undistort(pts, pts);
      for (size_t i = 0; i < src.size(); ++i) dst[i].pt = pts[i];
    }

    // 绘制归一化平面 (z=1)
    void draw_normalized_plane(const cv::Mat &src, cv::Mat &dst);

    // ORB 特征
    virtual void monoORBfeatures(ORB::Extractor *pExtractor, const cv::Mat &img,
                                 ORB::KeyPoints &kps, cv::Mat &desc) {
      pExtractor->detect_and_compute(img, cv::noArray(), kps, desc);
      undistort(kps, kps);
    }

#define STEREO_ORB_EXTRACT(lapCnt0, lapCnt1) \
  parallel::PriorityThread t0 = parallel::thread_pool.emplace( \
      1, [&lapCnt0, this, pExtractor0, &img0, &kps0, &desc0]() { \
          lapCnt0 = pExtractor0->detect_and_compute(img0, cv::noArray(), kps0, desc0); \
          this->undistort(kps0, kps0); \
      }); \
  lapCnt1 = pExtractor1->detect_and_compute(img1, cv::noArray(), kps1, desc1); \
  pCamRight->undistort(kps1, kps1); \
  t0->join(); \
  if (lapCnt1 == 0 || lapCnt0 == 0) return;

    virtual void stereoORBfeatures(Base *pCamRight,
                                   ORB::Extractor *pExtractor0, ORB::Extractor *pExtractor1,
                                   const cv::Mat &img0, const cv::Mat &img1,
                                   ORB::KeyPoints &kps0, ORB::KeyPoints &kps1,
                                   cv::Mat &desc0, cv::Mat &desc1, std::vector<cv::DMatch> &matches) = 0;
};

}

#endif
