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
  YAML::Node cfg = tumvi.loadCfg()["cam0"];

  Eigen::Vector4f dist = YAML::toEigen<float>(cfg["distortion_coeffs"]);
  Eigen::Matrix4f T_ci = YAML::toEigen<float>(cfg["T_cam_imu"]);

  std::cout << dist << std::endl << T_ci << std::endl;

  return 0;
}
