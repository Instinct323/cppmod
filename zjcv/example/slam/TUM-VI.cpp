#include <filesystem>

#include "dataset/tum_vi.hpp"
#include "cv.hpp"
#include "fbow.hpp"
#include "std.hpp"
#include "imu_type.hpp"
#include "logging.hpp"
#include "slam/system.hpp"


int main(int argc, char **argv) {
  putenv("DISPLAY=host.docker.internal:0");
  Logger logger(argv);

  // config
  YAML::Node cfg = YAML::LoadFile("/home/workbench/cppmod/ZJCV/example/slam/TUM-VI.yaml");
  slam::System system(cfg);

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

  // 载入词汇表
  std::string vocPath = cfg["vocabulary"].as<std::string>();
  fbow::Vocabulary voc;
  if (exists(std::filesystem::path(vocPath))) {
    voc.readFromFile(vocPath);
  } else {
    LOG(ERROR) << "Vocabulary not found: " << vocPath;
    LOG(INFO) << "Press any key to create a new vocabulary...";
    std::getchar();
    fbow::createORBvocabulary(voc, system.mpTracker->mpExtractor0, grayloader, vImgLeft);
    voc.saveToFile(vocPath);
  }

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
