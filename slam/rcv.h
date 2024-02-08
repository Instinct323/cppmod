#pragma once

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

typedef Sophus::SE3d SE3;
typedef Eigen::Matrix3d Mat33;
typedef Eigen::Vector3d Vec3;
typedef Eigen::Vector2d Vec2;


/** @brief 相机 */
class Camera {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Camera> Ptr;

    double fx = 1, fy = 1, cx = 0, cy = 0;
    SE3 pose, pose_inv;    // pcenter -> camera
    SE3 Tcw, Tcw_inv;   // camera -> world

    void set_pose(const SE3 &p) {
      pose = p;
      pose_inv = p.inverse();
    }

    void set_Tcw(const SE3 &T) {
      Tcw = T;
      Tcw_inv = T.inverse();
    }

    // 返回内参
    Mat33 K() const {
      Mat33 k;
      k << fx, 0, cx, 0, fy, cy, 0, 0, 1;
      return k;
    }

    // 坐标变换: world, camera, pixel
    Vec3 world2camera(const Vec3 &p_w) const {
      return pose * Tcw * p_w;
    }

    Vec3 camera2world(const Vec3 &p_c) const {
      return Tcw_inv * pose_inv * p_c;
    }

    Vec2 camera2pixel(const Vec3 &p_c) const {
      return {fx * p_c(0) / p_c(2) + cx,
              fy * p_c(1) / p_c(2) + cy};
    }

    Vec3 pixel2camera(const Vec2 &p_p, double depth) const {
      return {(p_p(0) - cx) / fx * depth,
              (p_p(1) - cy) / fy * depth,
              depth};
    }

    Vec2 world2pixel(const Vec3 &p_w) const { return camera2pixel(world2camera(p_w)); }

    Vec3 pixel2world(const Vec2 &p_p, double depth) const { return camera2world(pixel2camera(p_p, depth)); }
};


/**
 * @brief 基于 SVD 的线性三角剖分
 * @param poses - 相机位姿 (相对于机器人坐标系)
 * @param p_c - 相机坐标系下的关键点
 * @param p_r - 机器人坐标系下的关键点
 * @param z_floor - 地面高度
 */
bool triangulation(const std::vector<SE3> &poses,
                   const std::vector<Vec3> &p_c,
                   Vec3 &p_r,
                   double z_floor = 0.) {
  Eigen::MatrixXd equ_set(2 * p_c.size(), 4);
  for (int i = 0; i < p_c.size(); ++i) {
    // Ti * p_r = di * p_ci 等价:
    // 1. (Ti[0] - p_ci[0] * Ti[2]) * p_r = 0
    // 2. (Ti[1] - p_ci[1] * Ti[2]) * p_r = 0
    Eigen::Matrix<double, 3, 4> Ti = poses[i].matrix3x4();
    equ_set.block<2, 4>(2 * i, 0) = Ti.block<2, 4>(0, 0) - p_c[i].head(2) * Ti.row(2);
  }
  // A = USV^T, AV = US
  // 由于特征向量最后一个值最小, 故 AV 的最后一列趋近于零, 即 V 的最后一列为解
  auto svd = equ_set.bdcSvd(Eigen::ComputeThinV);
  p_r = svd.matrixV().col(3).head(3) / svd.matrixV()(3, 3);
  return (p_r[2] > z_floor && svd.singularValues()[3] / svd.singularValues()[2] < 1e-2);
}


/** @brief 图像对 */
class ImgPair {
public:
    cv::Mat img1, img2;

    ImgPair(cv::Mat &img1, cv::Mat &img2) : img1(img1), img2(img2) {}

    /** @brief LK 光流匹配关键点 */
    void match_keypoint(std::vector<cv::Point2f> &kp1,
                        std::vector<cv::Point2f> &kp2,
                        cv::Mat &status) const {
      cv::calcOpticalFlowPyrLK(
          img1, img2, kp1, kp2.empty() ? kp1 : kp2,
          status, cv::Mat(), cv::Size(11, 11), 3,
          cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
          cv::OPTFLOW_USE_INITIAL_FLOW);
    }
};
