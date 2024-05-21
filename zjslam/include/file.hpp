#ifndef ZJSLAM__FILE_HPP
#define ZJSLAM__FILE_HPP

#include <fstream>
#include <sophus/se3.hpp>
#include <yaml-cpp/yaml.h>

#include "extension/eigen.hpp"
#include "logging.hpp"

namespace CSV {

// 逐行映射
void rowMapping(const std::string &file, std::function<void(std::vector<std::string> &)> unary_op) {
  std::ifstream f(file);
  ASSERT(f.is_open(), "fail to open file " << file)
  // 读取文件, 除去空行和注释
  std::string line;
  while (std::getline(f, line)) {
    if (!line.empty() && line[0] != '#') {
      std::istringstream iss(line);
      std::string token;
      // 以逗号分隔读取
      std::vector<std::string> tokens;
      while (std::getline(iss, token, ',')) tokens.push_back(token);
      unary_op(tokens);
    }
  }
}
}


namespace TXT {

// 逐行映射
void rowMapping(const std::string &file, std::function<void(std::string &)> unary_op) {
  std::ifstream f(file);
  ASSERT(f.is_open(), "fail to open file " << file)
  // 读取文件, 除去空行和注释
  std::string line;
  while (std::getline(f, line)) {
    if (!line.empty() && line[0] != '#') unary_op(line);
  }
}
}


namespace YAML {

// std::vector 转换
template<typename T>
std::vector<T> toVec(const YAML::Node &node) {
  ASSERT(node.IsSequence(), "YAML: Invalid vector format");
  std::vector<T> vec;
  for (int i = 0; i < node.size(); ++i) vec.push_back(node[i].as<T>());
  return vec;
}


// 矩阵判断
void assertMatrix(const Node &node) {
  bool flag = node.IsSequence();
  if (flag) {
    // 校对每一行的列数
    int rows = node.size();
    int cols = node[0].size();
    if (cols) {
      for (int i = 0; i < rows; ++i) {
        flag = node[i].IsSequence() && node[i].size() == cols;
        if (!flag) break;
      }
    }
  }
  if (!flag) LOG(FATAL) << "YAML: Invalid matrix format";
}


// 矩阵转换
template<typename T, typename MatrixT>
MatrixT toMatrix(const Node &node) {
  assertMatrix(node);
  int rows = node.size(), cols = node[0].size();
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


// SE3d 转换
Sophus::SE3d toSE3d(const Node &node) {
  Eigen::MatrixXd mat = toEigen<double>(node);
  // tx ty tz qx qy qz qw
  if (mat.size() == 7) {
    return {Eigen::Quaterniond(mat(6), mat(3), mat(4), mat(5)),
            Eigen::Vector3d(mat(0), mat(1), mat(2))};
  } else if (mat.size() == 12 || mat.size() == 16) {
    // Rotation + Translation
    Eigen::MatrixXd matn4 = Eigen::reshape<double>(mat, -1, 4);
    return {Eigen::Quaterniond(matn4.block<3, 3>(0, 0)), matn4.block<3, 1>(0, 3)};
  }
  LOG(FATAL) << "YAML: Invalid SE3d format";
}

}

#endif
