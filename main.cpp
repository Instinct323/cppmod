#include <cstdlib>
#include <sophus/se3.hpp>

#include "utils/glog.hpp"


int main(int argc, char **argv) {
  putenv("DISPLAY=host.docker.internal:0");
  glog::Logger logger(argv);

  Eigen::Vector3f v(4, 2, 3);
  Eigen::Matrix3f v_hat = Sophus::SO3f::hat(v.normalized());
  float norm = v.norm();

  const Eigen::Matrix3f rightJ = Sophus::SO3f::leftJacobian(v);
  const Eigen::Matrix3f rightJ2 = rightJ;

  LOG(INFO) << rightJ;
  LOG(INFO) << rightJ2;

  return 0;
}
