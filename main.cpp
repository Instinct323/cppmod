#include <cstdlib>

#include "zjslam/include/logging.hpp"
#include "zjslam/include/dataset/tum_vi.hpp"
#include "zjslam/include/utils.hpp"
#include "zjslam/include/imu_type.hpp"
#include "zjslam/include/geometry.hpp"


void draft() {
  dataset::TumVI tumvi("/home/workbench/data/dataset-corridor4_512_16/dso");
  IMU::Device::Ptr dev = IMU::Device::fromYAML(tumvi.loadCfg("imu_config.yaml"));

  IMU::Bias bias(0.1, 0.1, 0.1, 0.1, 0.1, 0.1);
  LOG(INFO) << bias.a;
}


int main(int argc, char **argv) {
  putenv("DISPLAY=host.docker.internal:0");

  Logger logger(argv);
  LOG(INFO) << "CXX standard: " << __cplusplus;

  draft();
  return 0;
}
