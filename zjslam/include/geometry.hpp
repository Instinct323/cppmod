#ifndef ZJSLAM__GEOMETRY_HPP
#define ZJSLAM__GEOMETRY_HPP

#include <Eigen/Core>
#include <Eigen/SVD>

namespace Eigen {

// 旋转矩阵归一化
template<typename T>
Eigen::Matrix<T, 3, 3> normalize_rotation(const Eigen::Matrix<T, 3, 3> &R) {
  Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
  return svd.matrixU() * svd.matrixV().transpose();
}
}

#endif
