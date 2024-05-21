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

//  ORB::Extractor orb;
//  std::vector<cv::KeyPoint> keypoints;
//  cv::Mat descriptors;
//  orb(img, cv::noArray(), keypoints, descriptors);

  std::vector<int> x = {1, 2, 3, 4, 5};
  for (auto it = x.rbegin(); it != x.rend(); ++it) {
    LOG(INFO) << *it;
  }
}


int main(int argc, char **argv) {
  putenv("DISPLAY=host.docker.internal:0");

  Logger logger(argv);
  LOG(INFO) << "CXX standard: " << __cplusplus;

  draft();
  return 0;
}
