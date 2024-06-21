#ifndef ZJCV__ALGORITHM_HPP
#define ZJCV__ALGORITHM_HPP

#include <sophus/se3.hpp>


/** @brief 基于 SVD 的线性三角剖分
 *  @param Tcw - 相机位姿 (相对于世界坐标系)
 *  @param p_c - 相机坐标系下的关键点 */
bool triangulation(std::vector<Sophus::SE3d> &Tcw,
                   std::vector<Eigen::Vector3d> &p_c,
                   Eigen::Vector3d &p_w,
                   float z_floor = 0.) {
  if (p_c.size() < 2) {
    return false;
  } else {
    Eigen::MatrixXd equ_set(2 * p_c.size(), 4);
    for (int i = 0; i < p_c.size(); ++i) {
      // Ti * p_w = di * p_ci 等价:
      // 1. (Ti[0] - p_ci[0] * Ti[2]) * p_w = 0
      // 2. (Ti[1] - p_ci[1] * Ti[2]) * p_w = 0
      Eigen::Matrix<double, 3, 4> Ti = Tcw[i].matrix3x4();
      equ_set.block<2, 4>(2 * i, 0) = Ti.block<2, 4>(0, 0) - p_c[i].head(2) * Ti.row(2);
    }
    // A = USV^T, AV = US
    // 由于特征向量最后一个值最小, 故 AV 的最后一列趋近于零, 即 V 的最后一列为解
    auto svd = equ_set.bdcSvd(Eigen::ComputeThinV);
    p_w = svd.matrixV().col(3).head(3) / svd.matrixV()(3, 3);
    return p_w[2] > z_floor && svd.singularValues()[3] / svd.singularValues()[2] < 1e-2;
  }
}

#endif
