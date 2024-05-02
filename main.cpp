#include <cstdlib>

#include "zjslam/include/logging.hpp"
#include "zjslam/include/utils.hpp"
#include "zjslam/include/dataset/tum_vi.hpp"


int main(int argc, char **argv) {
  putenv("DISPLAY=host.docker.internal:0");

  Logger logger(argv);
  LOG(INFO) << "CXX standard: " << __cplusplus;

  std::cout.precision(6);
  // cv::remap();
  // cv::initUndistortRectifyMap();
  dataset::TumVI tumvi("/home/workbench/data/dataset-corridor4_512_16/dso");
  std::cout << YAML::toCvMat<float>(tumvi.loadCfg()["cam0"]["T_cam_imu"]);

  return 0;
}
