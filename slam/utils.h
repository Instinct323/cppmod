#pragma once

#include <chrono>
#include <Eigen/Core>
#include <glog/logging.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

typedef Sophus::SE3d SE3;
typedef Eigen::Matrix3d Mat33;
typedef Eigen::Vector3d Vec3;
typedef Eigen::Vector2d Vec2;


/** @brief 日志 */
class Logger {
public:
    explicit Logger(char **argv) {
      google::InitGoogleLogging(argv[0]);
      FLAGS_logtostderr = true;
      FLAGS_minloglevel = google::INFO;
    }

    ~Logger() { google::ShutdownGoogleLogging(); }
};


/** @brief 计时器 */
class Timer {

public:
    typedef std::chrono::steady_clock Clock;
    typedef Clock::time_point Timepoint;
    typedef std::chrono::duration<double> Duration;

    Timepoint t0;

    Timer() { t0 = Clock::now(); }

protected:
    friend std::ostream &operator<<(std::ostream &os, const Timer &timer) {
      Timepoint t1 = Clock::now();
      Duration time_used = std::chrono::duration_cast<Duration>(t1 - timer.t0);
      return (os << time_used.count());
    }
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
