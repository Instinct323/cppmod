#include "utils/eigen.hpp"

namespace Eigen {

bool triangulation(std::vector<Eigen::Vector3d> &vP_cam,
                   std::vector<Sophus::SE3d> &vT_cam_ref,
                   Eigen::Vector3d &P_ref,
                   float &reproj_error) {
  size_t n = vP_cam.size();
  if (n < 2) return false;
  std::vector<Eigen::Vector4d> vT_cf_row2;
  Eigen::MatrixXd equ_set(2 * n, 4);
  for (int i = 0; i < n; ++i) {
    // T_cam_ref * P_ref = z * P_cam 等价:
    // 1. (T_cam_ref[0] - x * T_cam_ref[2]) * P_ref = 0
    // 2. (T_cam_ref[1] - y * T_cam_ref[2]) * P_ref = 0
    Eigen::Matrix<double, 3, 4> T_cam_ref = vT_cam_ref[i].matrix3x4();
    vT_cf_row2.push_back(T_cam_ref.row(2));
    equ_set.block<2, 4>(2 * i, 0) = T_cam_ref.block<2, 4>(0, 0) - vP_cam[i].head(2) * T_cam_ref.row(2);
  }
  // A[2n, 4]: n 个 (T_cam_ref[:2] - p2D * T_cam_ref[2])
  // V[4, 4]: AV = US, 每一列对应一个可能的 P_ref
  // 由于特征向量 S 最后一个值最小, 故 US (即 AV) 的最后一列趋近于零, 即 V 的最后一列为解
  auto svd = equ_set.bdcSvd(Eigen::ComputeThinV);
  Eigen::Vector4d P_homo = svd.matrixV().col(3);
  P_homo /= P_homo[3];
  // if (svd.singularValues()[3] / svd.singularValues()[2] > 1e-2) return false;
  // 检验点在相机平面上的深度
  for (int i = 0; i < vT_cf_row2.size(); ++i) {
    if (vT_cf_row2[i].dot(P_homo) <= 0) return false;
  }
  P_ref = P_homo.head(3);
  reproj_error = (equ_set * P_homo).cwiseAbs().mean();
  return true;
}

}
