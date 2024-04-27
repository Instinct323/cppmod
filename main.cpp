#include "utils.hpp"

#include "zjslam/include/camera/pinhole.hpp"
#include "zjslam/include/camera/kannala_brandt.hpp"


int main(int argc, char **argv) {
  Logger logger(argv);
  LOG(INFO) << "CXX standard: " << __cplusplus;

  Pinhole cam1({1, 2, 3, 3});
  LOG(INFO) << cam1.projectEig(cv::Point3f(1, 2, 3));
  LOG(INFO) << atan(10 / 0.f);

  return 0;
}
