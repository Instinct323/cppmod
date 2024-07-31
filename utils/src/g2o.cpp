#include "utils/g2o.hpp"

namespace g2o {

Sophus::SE3f toSE3(const g2o::SE3Quat &pose) {
  Eigen::Matrix4f mat = pose.to_homogeneous_matrix().cast<float>();
  return Sophus::SE3f(mat);
}

}
