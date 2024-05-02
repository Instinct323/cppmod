#ifndef ZJSLAM__FILE_HPP
#define ZJSLAM__FILE_HPP

#include <fstream>
#include <yaml-cpp/yaml.h>

#include "utils.hpp"


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

// 矩阵判断
void assertMatrix(const Node &node) {
  bool flag = node.IsSequence();
  if (flag) {
    // 校对每一行的列数
    int rows = node.size();
    int cols = node[0].size();
    for (int i = 0; i < rows; ++i) {
      flag = node[i].IsSequence() && node[i].size() == cols;
      if (!flag) break;
    }
  }
  if (!flag) LOG(FATAL) << "YAML: Invalid matrix format";
}


// Eigen 转换
template<typename T>
Eigen::Matrix<T, -1, -1> toEigen(const Node &node) {
  assertMatrix(node);
  Eigen::Matrix<T, -1, -1> mat(node.size(), node[0].size());
  for (int i = 0; i < mat.rows(); ++i) {
    for (int j = 0; j < mat.cols(); ++j) mat(i, j) = node[i][j].as<T>();
  }
  return mat;
}

// cv::Mat 转换
template<typename T>
cv::Mat_<T> toCvMat(const Node &node) {
  assertMatrix(node);
  cv::Mat_<T> mat(node.size(), node[0].size());
  for (int i = 0; i < mat.rows; ++i) {
    for (int j = 0; j < mat.cols; ++j) mat(i, j) = node[i][j].as<T>();
  }
  return mat;
}
}

#endif
