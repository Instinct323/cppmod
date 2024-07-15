#include <cstdlib>
#include <sophus/se3.hpp>

#include "utils/glog.hpp"


int main(int argc, char **argv) {
  putenv("DISPLAY=host.docker.internal:0");
  glog::Logger logger(argv);

  Eigen::Vector3f v(0, 1, 3);
  Eigen::Vector3f w(0, 0.02, 0);

  LOG(INFO) << (Sophus::SO3f::exp(w) * Sophus::SO3f::exp(v)).matrix();
  LOG(INFO) << (Sophus::SO3f::leftJacobian(w) * Sophus::SO3f::exp(v).matrix());

  return 0;
}
