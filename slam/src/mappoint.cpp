#include "frame.h"
#include "mappoint.h"


void Mappoint::triangulation() {
  std::vector<SE3> Tcw;   // 相机位姿 (相对于世界坐标系)
  std::vector<Vec3> p_c;  // 相机坐标系下的关键点

  for (int i = kps.size() - 1; i >= 0; i--) {
    auto it = kps.begin() + i;
    auto frame = kps[i].first.lock();
    // 去除失效的关键点
    if (frame == nullptr) {
      kps.erase(it);
    } else if (frame->get_Tcw(Tcw)) {
      // 相机坐标系下的关键点
      Vec2 p_p = frame->kps[kps[i].second];
      p_c.push_back(frame->camera->pixel2camera(p_p, 1));
    }
  }

  if (p_c.size() < 2) {
    is_inlier = false;
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
    is_inlier = p_w[2] > z_floor && svd.singularValues()[3] / svd.singularValues()[2] < 1e-2;
  }
}
