#ifndef UTILS__EIGEN_HPP
#define UTILS__EIGEN_HPP

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

#include "glog.hpp"

namespace Eigen {

/** @brief 基于 SVD 的线性三角剖分
 *  @param vP_cam - 相机坐标系下的关键点
 *  @param vT_cam_ref - 相机位姿 (相对于参考坐标系) */
bool triangulation(std::vector<Eigen::Vector3f> &vP_cam,
                   std::vector<Sophus::SE3f> &vT_cam_ref,
                   Eigen::Vector3f &P_ref,
                   float &reproj_error);


// cosine
template<typename T, int dim>
T cos(Eigen::Matrix<T, dim, 1> &v1, Eigen::Matrix<T, dim, 1> &v2) {
  return v1.dot(v2) / (v1.norm() * v2.norm());
}


// Eigen -> cv::Mat
template<typename T>
cv::Mat toCvMat(const Matrix<T, -1, -1> &m) {
  cv::Mat_<T> mat(m.rows(), m.cols());
  for (int i = 0; i < mat.rows; i++) {
    for (int j = 0; j < mat.cols; j++) {
      mat.template at<T>(i, j) = m(i, j);
    }
  }
  return mat;
}


// Eigen reshape
template<typename T>
Matrix<T, -1, -1> reshape(Matrix<T, -1, -1> &m, int rows, int cols) {
  ASSERT(rows > 0 || cols > 0, "Invalid reshape size")
  if (rows == -1) rows = m.size() / cols;
  if (cols == -1) cols = m.size() / rows;
  ASSERT(rows * cols == m.size(), "Invalid reshape size")
  Matrix<T, -1, -1> newMat(rows, cols);
  for (int i = 0; i < newMat.size(); ++i) newMat(i) = m(i);
  return newMat;
}


// 旋转矩阵归一化
template<typename T>
Eigen::Matrix<T, 3, 3> normalize_rotation(const Eigen::Matrix<T, 3, 3> &R) {
  Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
  return svd.matrixU() * svd.matrixV().transpose();
}

}

#endif
