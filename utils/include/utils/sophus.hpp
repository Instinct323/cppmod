#ifndef UTILS__SOPHUS_HPP
#define UTILS__SOPHUS_HPP

#include <sophus/se3.hpp>
#include <sophus/sim3.hpp>

namespace Sophus {

/** @brief 基于 SVD 的线性三角剖分
 *  @param vP_cam - 相机坐标系下的关键点
 *  @param vT_cam_ref - 相机位姿 (相对于参考坐标系) */
float triangulation(const std::vector<Eigen::Vector3f> &vP_cam,
                    const std::vector<Sophus::SE3f> &vT_cam_ref,
                    Eigen::Vector3f &P_ref);

/** @brief 基于单位四元数的轨迹对齐
 *  @return 适用于 pts1 的相似变换 Sim3 */
Sophus::Sim3f align_trajectory(const std::vector<Eigen::Vector3f> &pts1,
                               const std::vector<Eigen::Vector3f> &pts2);

// 绝对位移误差 (RMSE)
double abs_trans_error(const std::vector<Eigen::Vector3f> &pts1,
                       const std::vector<Eigen::Vector3f> &pts2);


template<typename T>
Eigen::Matrix3<T> rightJacobian(const Eigen::Vector3<T> &omega) {
  T theta2 = omega.squaredNorm(), eps = Sophus::Constants<T>::epsilon();
  Eigen::Matrix3<T> rightJ = Eigen::Matrix3<T>::Identity();

  if (theta2 > eps * eps) {
    T theta = std::sqrt(theta2);
    Eigen::Matrix3<T> omega_hat = SO3<T>::hat(omega), omega_hat2 = omega_hat * omega_hat;
    rightJ += -(1 - std::cos(theta)) * omega_hat / theta2
              + (theta - std::sin(theta)) * omega_hat2 / (theta2 * theta);
  }
  return rightJ;
}

}

#endif
