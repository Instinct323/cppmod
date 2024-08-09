#ifndef UTILS__SOPHUS_HPP
#define UTILS__SOPHUS_HPP

#include <sophus/se3.hpp>
#include <sophus/sim3.hpp>
#include <g2o/types/slam3d/se3quat.h>

namespace Sophus {

// Sophus -> g2o
g2o::SE3Quat toG2O(const Sophus::SE3f &T);

/** @brief 基于 SVD 的线性三角剖分
 *  @param vP_cam - 相机坐标系下的关键点 (z=1)
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


// 右雅可比矩阵
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


// 关节
class Joint {
    const Sophus::SE3f *T_world_ref;
    Sophus::SE3f T_ref_cur;

public:
    explicit Joint(const Sophus::SE3f *T_world_ref,
                   const Sophus::SE3f &T_ref_cur = Sophus::SE3f()) : T_world_ref(T_world_ref), T_ref_cur(T_ref_cur) {}

    void set(const Sophus::SE3f &T_rc) { this->T_ref_cur = T_rc; }

    // T_world_cur
    Sophus::SE3f get() const { return T_ref_cur * (*T_world_ref); }
};


// 重载输出
template<typename T>
std::ostream &operator<<(std::ostream &os, const Sophus::SE3<T> &se3) {
  const Eigen::Quaternion<T> &q = se3.unit_quaternion();
  const Eigen::Vector3<T> &t = se3.translation();
  return (os << t[0] << " " << t[1] << " " << t[2] << " " <<
             q.w() << " " << q.x() << " " << q.y() << " " << q.z());
}

}

#endif
