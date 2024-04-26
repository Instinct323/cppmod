#include "utils.hpp"

#define ROOT "../data"

#include "zjslam/include/camera/pinhole.hpp"


int main(int argc, char **argv) {
  Logger logger(argv);
  LOG(INFO) << __cplusplus;

  Pinhole cam1({1, 2, 3, 3});
  LOG(INFO) << cam1.projectMat(cv::Point3f(1, 2, 3));

  return 0;
}
