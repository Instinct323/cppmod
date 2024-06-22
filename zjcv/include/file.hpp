#ifndef ZJCV__FILE_HPP
#define ZJCV__FILE_HPP

#include <fstream>
#include <sophus/se3.hpp>
#include <yaml-cpp/yaml.h>

#include "extension/eigen.hpp"
#include "logging.hpp"

namespace CSV {

// 逐行映射
void rowMapping(const std::string &file, const std::function<void(std::vector<std::string> &)> &unary_op);

}


namespace TXT {

// 逐行映射
void rowMapping(const std::string &file, const std::function<void(std::string &)> &unary_op);

}


namespace YAML {

// SE3d 转换
Sophus::SE3d toSE3d(const Node &node);

// 矩阵判断
void assertMatrix(const Node &node);


// std::vector 转换
template<typename T>
std::vector<T> toVec(const YAML::Node &node) {
  ASSERT(node.IsSequence(), "YAML: Invalid vector format")
  std::vector<T> vec;
  for (int i = 0; i < node.size(); ++i) vec.push_back(node[i].as<T>());
  return vec;
}


// 矩阵转换
template<typename T, typename MatrixT>
MatrixT toMatrix(const Node &node) {
  assertMatrix(node);
  size_t rows = node.size(), cols = node[0].size();
  MatrixT mat(rows, MAX(1, cols));
  // 向量形式
  if (cols == 0) {
    for (int i = 0; i < rows; ++i) mat(i) = node[i].as<T>();
  } else {
    // 矩阵形式
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) mat(i, j) = node[i][j].as<T>();
    }
  }
  return mat;
}


// Eigen 转换
template<typename T>
Eigen::Matrix<T, -1, -1> toEigen(const Node &node) { return toMatrix<T, Eigen::Matrix<T, -1, -1>>(node); }

// cv::Mat 转换
template<typename T>
cv::Mat toCvMat(const Node &node) { return toMatrix<T, cv::Mat_<T>>(node); }

}

#endif
