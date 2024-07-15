#ifndef UTILS__SOPHUS_HPP
#define UTILS__SOPHUS_HPP

#include <sophus/se3.hpp>

namespace Sophus {

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
