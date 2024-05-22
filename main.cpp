#include <cstdlib>

#include "zjslam/include/logging.hpp"
#include "zjslam/include/dataset/tum_vi.hpp"
#include "zjslam/include/utils.hpp"
#include "zjslam/include/imu_type.hpp"
#include "zjslam/include/extension/cv.hpp"
#include "zjslam/include/extension/orb.hpp"

#include <opencv2/opencv.hpp>


void draft() {
  dataset::TumVI tumvi("/home/workbench/data/dataset-corridor4_512_16/dso");
  IMU::Device::Ptr dev = IMU::Device::fromYAML(tumvi.loadCfg("imu_config.yaml"));

  dataset::Filenames vFilename;
  dataset::Timestamps vTimestamps;
  tumvi.loadImage(vTimestamps, vFilename);

  cv::GrayLoader loader;
  cv::Mat img = loader(vFilename[0]);

  ORB::Extractor orb = ORB::Extractor(200, 1.2f, 8, {300, 400});
  ORB::KeyPoints keypoints;
  cv::Mat descriptors;

  int monoCnt = orb(img, cv::noArray(), keypoints, descriptors);
  std::cout << "begin: ";
  for (auto &kp : keypoints) {
    std::cout << kp.pt.x << " ";
  }
}


int main(int argc, char **argv) {
  putenv("DISPLAY=host.docker.internal:0");

  Logger logger(argv);
  LOG(INFO) << "CXX standard: " << __cplusplus;

  draft();
  return 0;
}
