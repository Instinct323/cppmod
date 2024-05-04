#ifndef ZJSLAM__CONVERT_HPP
#define ZJSLAM__CONVERT_HPP

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include "logging.hpp"

namespace cvt {

template<typename T>
Eigen::Matrix<T, -1, -1> reshape(Eigen::Matrix<T, -1, -1> &m, int rows, int cols) {
  ASSERT(rows != -1 || cols != -1, "Invalid reshape size")
  if (rows == -1) rows = m.size() / cols;
  if (cols == -1) cols = m.size() / rows;
  ASSERT(rows * cols == m.size(), "Invalid reshape size")
  Eigen::Matrix<T, -1, -1> newMat(rows, cols);
  for (int i = 0; i < newMat.size(); ++i) newMat(i) = m(i);
  return newMat;
}


template<typename T>
cv::Mat toCvMat(const Eigen::Matrix<T, -1, -1> &m) {
  cv::Mat_<T> mat(m.rows(), m.cols());
  for (int i = 0; i < mat.rows; i++) {
    for (int j = 0; j < mat.cols; j++) {
      mat.template at<T>(i, j) = m(i, j);
    }
  }
  return mat;
}


template<typename T>
Eigen::Matrix<T, -1, -1> toEigen(const cv::Mat &mat) {
  Eigen::Matrix<T, -1, -1> m(mat.rows, mat.cols);
  for (int i = 0; i < m.rows(); i++) {
    for (int j = 0; j < m.cols(); j++) {
      m(i, j) = mat.at<T>(i, j);
    }
  }
  return m;
}


template<typename T>
Eigen::Vector<T, -1> toEigen(const std::vector<T> &vec) {
  Eigen::Vector<T, -1> v(vec.size());
  for (int i = 0; i < vec.size(); i++) v(i) = vec[i];
  return v;
}
}

#endif
