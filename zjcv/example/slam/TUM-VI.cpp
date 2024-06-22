#include "dataset/tum_vi.hpp"
#include "extension/cv.hpp"
#include "extension/std.hpp"
#include "imu_type.hpp"
#include "logging.hpp"
#include "slam/system.hpp"


int main(int argc, char **argv) {
  putenv("DISPLAY=host.docker.internal:0");
  Logger logger(argv);
  YAML::Node cfg = YAML::LoadFile("/home/workbench/cppmod/ZJCV/example/slam/TUM-VI.yaml");

  // dataset
  dataset::Timestamps vImgLeftTs, vImgRightTs, vImuTs;
  dataset::Filenames vImgLeft, vImgRight;
  dataset::IMUsamples vImu;

  // 载入并校验数据
  dataset::TumVI tum_vi(cfg["dataset"].as<std::string>());
  tum_vi.loadImage(vImgLeftTs, vImgLeft, "cam0");
  tum_vi.loadImage(vImgRightTs, vImgRight, "cam1");
  tum_vi.loadIMU(vImuTs, vImu);
  assert(vImgLeftTs.size() == vImgRightTs.size());
  LOG(INFO) << "Images: " << vImgLeftTs.size() << ", IMU: " << vImuTs.size();

  cv::GrayLoader grayloader;
  std::ValueSlicer<double> slicer(vImuTs);
  slam::System system(cfg);

  // main loop
  for (size_t i = 0; i < vImgLeftTs.size(); i++) {
    cv::Mat imgLeft = grayloader(vImgLeft[i]), imgRight = grayloader(vImgRight[i]);
    auto [j, k] = slicer(vImgLeftTs[i]);

    system.GrabStereo(vImgLeftTs[i], imgLeft, imgRight,
                      dataset::IMUsamples(vImu.begin() + j, vImu.begin() + k));

    cv::imshow("Left", imgLeft);
    cv::imshow("Right", imgRight);
    cv::waitKey(1);
  }

  return 0;
}
