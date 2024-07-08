#include <boost/format.hpp>
#include <filesystem>

#include "system.hpp"
#include "utils/cv.hpp"
#include "utils/fbow.hpp"
#include "utils/indicators.hpp"
#include "utils/glog.hpp"
#include "utils/math.hpp"
#include "zjcv/dataset/tum_vi.hpp"
#include "zjcv/imu.hpp"

int main(int argc, char **argv) {
  putenv("DISPLAY=host.docker.internal:0");
  glog::Logger logger(argv);

  // config
  YAML::Node cfg = YAML::LoadFile("/home/workbench/cppmod/ORB-slam3/example/TUM-VI.yaml");
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
  math::ValueSlicer<double> slicer(vImuTs);

  // 载入词汇表
  std::string vocPath = cfg["vocabulary"].as<std::string>();
  fbow::Vocabulary voc;
  if (exists(std::filesystem::path(vocPath))) {
    voc.readFromFile(vocPath);
  } else {
    LOG(ERROR) << "Vocabulary not found: " << vocPath;
    LOG(INFO) << "Press any key to create a new vocabulary...";
    std::getchar();
    fbow::createORBvocabulary(voc, system.mpTracker->mpExtractor0.get(), grayloader, vImgLeft);
    voc.saveToFile(vocPath);
  }

  // main loop
  system.run();
  auto pbar = indicators::getProgressBar(vImgLeftTs.size());
  glog::Timer timer;
  boost::format fmt("Cost=%.1fms");

  while (!pbar.is_completed()) {
    int i = pbar.current();
    cv::Mat imgLeft = grayloader(vImgLeft[i]), imgRight = grayloader(vImgRight[i]);
    auto [j, k] = slicer(vImgLeftTs[i]);

    timer.reset();
    system.grab_imu(vImgLeftTs[i],
                    dataset::Timestamps(vImuTs.begin() + j, vImuTs.begin() + k),
                    dataset::IMUsamples(vImu.begin() + j, vImu.begin() + k));
    system.grab_stereo(vImgLeftTs[i], imgLeft, imgRight);
    indicators::set_desc(pbar, (fmt % (1e3 * timer.count())).str(), false);
    pbar.tick();
  }
  pbar.mark_as_completed();
  system.stop();

  return 0;
}
