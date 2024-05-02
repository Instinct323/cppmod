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
cv::Mat toCvMat(const Node &node) {return toMatrix<T, cv::Mat_<T>>(node);}

}

#endif
