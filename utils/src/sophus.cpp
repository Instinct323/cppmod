#include "utils/glog.hpp"
#include "utils/sophus.hpp"

namespace Sophus {


bool triangulation(const std::vector<Eigen::Vector3f> &vP_cam,
                   const std::vector<Sophus::SE3f> &vT_cam_ref,
                   Eigen::Vector3f &P_ref,
                   float &reproj_error) {
  size_t n = vP_cam.size();
  if (n < 2) return false;
  Eigen::MatrixXf equ_set(2 * n, 4);
  for (int i = 0; i < n; ++i) {
    // T_cam_ref * P_ref = z * P_cam 等价:
    // 1. (T_cam_ref[0] - x * T_cam_ref[2]) * P_ref = 0
    // 2. (T_cam_ref[1] - y * T_cam_ref[2]) * P_ref = 0
    Eigen::Matrix<float, 3, 4> T_cam_ref = vT_cam_ref[i].matrix3x4();
    equ_set.block<2, 4>(2 * i, 0) = T_cam_ref.block<2, 4>(0, 0) - vP_cam[i].head(2) * T_cam_ref.row(2);
  }
  // A[2n, 4]: n 个 (T_cam_ref[:2] - p2D * T_cam_ref[2])
  // V[4, 4]: AV = US, 每一列对应一个可能的 P_ref
  // 由于特征向量 S 最后一个值最小, 故 US (即 AV) 的最后一列趋近于零, 即 V 的最后一列为解
  auto svd = equ_set.bdcSvd(Eigen::ComputeThinV);
  Eigen::Vector4f P_homo = svd.matrixV().col(3);
  P_homo /= P_homo[3];
  P_ref = P_homo.head(3);
  // if (svd.singularValues()[3] / svd.singularValues()[2] > 1e-2) return false;
  // 检验点在相机平面上的深度
  reproj_error = 0;
  for (int i = 0; i < n; ++i) {
    Eigen::Vector3f P_cam = vT_cam_ref[i] * P_ref;
    if (P_cam[2] < 1e-4) return false;
    reproj_error = std::max(reproj_error, ((P_cam.head(2) / P_cam[2]) - vP_cam[i].head(2)).squaredNorm());
  }
  return true;
}


Sophus::Sim3f align_trajectory(const std::vector<Eigen::Vector3f> &pts1,
                               const std::vector<Eigen::Vector3f> &pts2) {
  assert(pts1.size() == pts2.size());
  int n = pts1.size();
  // 计算质心
  Eigen::Vector3d centroid1 = Eigen::Vector3d::Zero(), centroid2 = Eigen::Vector3d::Zero();
  for (int i = 0; i < n; ++i) {
    Eigen::Vector3d p1 = pts1[i].cast<double>(), p2 = pts2[i].cast<double>();
    centroid1 += p1;
    centroid2 += p2;
  }
  centroid1 /= n;
  centroid2 /= n;
  // 零均值化, 计算尺度因子、lut
  Eigen::Matrix3<long double> lut = Eigen::Matrix3<long double>::Zero();
  long double Sl = 0, Sr = 0;
  for (int i = 0; i < n; ++i) {
    Eigen::Vector3d p1 = pts1[i].cast<double>() - centroid1, p2 = pts2[i].cast<double>() - centroid2;
    // 如果 pts 数值较大, 可能会溢出
    lut += (p1 * p2.transpose()).cast<long double>();
    Sl += p1.squaredNorm();
    Sr += p2.squaredNorm();
  }
  lut /= n;
  double Sxx = lut(0, 0), Sxy = lut(0, 1), Sxz = lut(0, 2),
      Syx = lut(1, 0), Syy = lut(1, 1), Syz = lut(1, 2),
      Szx = lut(2, 0), Szy = lut(2, 1), Szz = lut(2, 2);
  double s = std::sqrt(Sr / Sl);
  // 单位四元数
  Eigen::Matrix4d N;
  N << (Sxx + Syy + Szz) / 2, Syz - Szy, Szx - Sxz, Sxy - Syx,
      0, (Sxx - Syy - Szz) / 2, Syx + Sxy, Szx + Sxz,
      0, 0, (-Sxx + Syy - Szz) / 2, Szy + Syz,
      0, 0, 0, (-Sxx - Syy + Szz) / 2;
  N += N.transpose().eval();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eig(N);
  Eigen::Vector4d q = eig.eigenvectors().col(3);
  // 代入数值
  float w = q[0], x = q[1], y = q[2], z = q[3];
  Eigen::Matrix3f R;
  R << w * w + x * x - y * y - z * z, -2 * w * z + 2 * x * y, 2 * w * y + 2 * x * z,
      2 * w * z + 2 * x * y, w * w - x * x + y * y - z * z, -2 * w * x + 2 * y * z,
      -2 * w * y + 2 * x * z, 2 * w * x + 2 * y * z, w * w - x * x - y * y + z * z;
  Eigen::Vector3f t = centroid2.cast<float>() - s * R * centroid1.cast<float>();
  return Sophus::Sim3f(s, Eigen::Quaternionf(R), t);
}


double abs_trans_error(const std::vector<Eigen::Vector3f> &pts1,
                       const std::vector<Eigen::Vector3f> &pts2) {
  assert(pts1.size() == pts2.size());
  long double e = 0;
  for (int i = 0; i < pts1.size(); ++i) e += (pts1[i] - pts2[i]).squaredNorm();
  return std::sqrt(e / pts1.size());
}

}
