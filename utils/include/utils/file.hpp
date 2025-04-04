#ifndef ZJCV__FILE_HPP
#define ZJCV__FILE_HPP

#include <sophus/se3.hpp>
#include <yaml-cpp/yaml.h>

#include "eigen.hpp"

namespace CSV {

// 逐行映射
void row_mapping(const std::string &file, const std::function<void(std::vector<std::string> &)> &unary_op);

}


namespace TXT {

// 逐行映射
void row_mapping(const std::string &file, const std::function<void(std::string &)> &unary_op);

}


namespace YAML {

// 结点有效性
bool is_invalid(const Node &node);

// 矩阵判断
void assert_matrix(const Node &node);


// std::vector 转换
template <typename T>
std::vector<T> toVec(const YAML::Node &node) {
    assert(node.IsSequence() && "YAML: Invalid vector format");
    std::vector<T> vec;
    for (const auto &x: node) vec.push_back(x.as<T>());
    return vec;
}


// 矩阵转换
template <typename T, typename MatrixT>
MatrixT toMatrix(const Node &node) {
    assert_matrix(node);
    int rows = node.size(), cols = node[0].size();
    MatrixT mat(rows, std::max(1, cols));
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
template <typename T>
Eigen::Matrix<T, -1, -1> toEigen(const Node &node) { return toMatrix<T, Eigen::Matrix<T, -1, -1>>(node); }


// cv::Mat 转换
template <typename T>
cv::Mat toCvMat(const Node &node) { return toMatrix<T, cv::Mat_<T>>(node); }


// Sophus 转换
template <typename T>
Sophus::SE3<T> toSE3(const Node &node) {
    Eigen::MatrixX<T> mat = toEigen<T>(node);
    // tx ty tz qw qx qy qz
    if (mat.size() == 7) {
        return {
            Eigen::Quaternion<T>(mat(3), mat(4), mat(5), mat(6)),
            Eigen::Vector3<T>(mat(0), mat(1), mat(2))
        };
    } else if (mat.size() == 12 || mat.size() == 16) {
        // Rotation + Translation
        Eigen::MatrixX<T> matn4 = Eigen::reshape<T>(mat, -1, 4);
        return {Eigen::Quaternion<T>(matn4.template block<3, 3>(0, 0)), matn4.template block<3, 1>(0, 3)};
    }
    throw std::invalid_argument("YAML: Invalid SE3 format");
}

}

#endif
