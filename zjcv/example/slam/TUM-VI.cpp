#include <filesystem>

#include "dataset/tum_vi.hpp"
#include "imu_type.hpp"
#include "slam/system.hpp"
#include "utils/cv.hpp"
#include "utils/fbow.hpp"
#include "utils/indicators.hpp"
#include "utils/logging.hpp"
#include "utils/std.hpp"


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
  tum_vi.load_image(vImgLeftTs, vImgLeft, "cam0");
  tum_vi.load_image(vImgRightTs, vImgRight, "cam1");
  tum_vi.load_imu(vImuTs, vImu);
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
  system.run();
  auto pbar = indicators::getProgressBar(vImgLeftTs.size());
  while (!pbar.is_completed()) {
    int i = pbar.current();
    cv::Mat imgLeft = grayloader(vImgLeft[i]), imgRight = grayloader(vImgRight[i]);
    auto [j, k] = slicer(vImgLeftTs[i]);

    system.grad_stereo(vImgLeftTs[i], imgLeft, imgRight,
                       dataset::IMUsamples(vImu.begin() + j, vImu.begin() + k));
    pbar.tick();
  }
  pbar.mark_as_completed();
  system.stop();

  return 0;
}
