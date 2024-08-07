#include <boost/format.hpp>
#include <filesystem>

#include "utils/cv.hpp"
#include "utils/fbow.hpp"
#include "utils/indicators.hpp"
#include "utils/glog.hpp"
#include "utils/math.hpp"
#include "zjcv/dataset/tum_vi.hpp"
#include "zjcv/imu.hpp"
#include "zjcv/slam.hpp"

std::tuple<
    dataset::Timestamps, dataset::Filenames,
    dataset::Timestamps, dataset::Filenames,
    dataset::Timestamps, dataset::IMUsamples,
    dataset::Timestamps, dataset::Poses> storage;


// Tracking
void slam::Tracker::run() {
  glog::Timer timer;
  cv::GrayLoader grayloader;
  math::ValueSlicer<double> slicer(&std::get<4>(storage));
  auto pbar = indicators::getProgressBar(std::get<0>(storage).size());

  while (!pbar.is_completed()) {
    int i = pbar.current();
    cv::Mat imgLeft = grayloader(std::get<1>(storage)[i]), imgRight = grayloader(std::get<3>(storage)[i]);
    // auto [j, k] = slicer(std::get<0>(storage)[i]);

    // 读入数据
    timer.reset();
    // grab_imu(std::get<0>(storage)[i],
    //          dataset::Timestamps(std::get<4>(storage).begin() + j, std::get<4>(storage).begin() + k),
    //          dataset::IMUsamples(std::get<5>(storage).begin() + j, std::get<5>(storage).begin() + k));
    grab_image(std::get<0>(storage)[i], imgLeft, imgRight);

    mpSystem->set_desc("track-cost", (boost::format("%.1fms") % (1e3 * timer.count())).str());
    mpSystem->set_desc("state", mState);
    indicators::set_desc(pbar, mpSystem->get_desc(), false);
    pbar.tick();
  }

  // 写入位姿
  /* LOG(INFO) << "Wait for writing pose...";
  std::ofstream ofs("pose.txt");
  for (const auto [ts, joint]: joints)
    ofs << std::fixed << size_t(ts * 1e9) << " " << joint.get() << std::endl;
  ofs.close(); */
}


int main(int argc, char **argv) {
  putenv("DISPLAY=host.docker.internal:0");
  glog::Logger logger(argv);
  LOG(INFO) << "Thread pool size: " << parallel::thread_pool_size;

  // config
  YAML::Node cfg = YAML::LoadFile("/home/workbench/cppmod/ORB-slam3/example/TUM-VI.yaml");
  slam::System system(cfg);
  system.mpTracker->reload(cfg["tracker"]);

  // 载入并校验数据
  dataset::TumVI tum_vi(cfg["dataset"].as<std::string>());
  tum_vi.load_image(std::get<0>(storage), std::get<1>(storage), "cam0");
  tum_vi.load_image(std::get<2>(storage), std::get<3>(storage), "cam1");
  tum_vi.load_imu(std::get<4>(storage), std::get<5>(storage));
  tum_vi.load_pose(std::get<6>(storage), std::get<7>(storage));

  assert(std::get<0>(storage).size() == std::get<2>(storage).size());
  LOG(INFO) << "Images: " << std::get<0>(storage).size()
            << ", IMU: " << std::get<4>(storage).size()
            << ", Poses: " << std::get<6>(storage).size();

  // 载入词汇表
  std::string vocPath = cfg["vocabulary"].as<std::string>();
  fbow::Vocabulary voc;
  if (exists(std::filesystem::path(vocPath))) {
    voc.readFromFile(vocPath);
  } else {
    LOG(ERROR) << "Vocabulary not found: " << vocPath;
    LOG(INFO) << "Press any key to create a new vocabulary...";
    std::getchar();
    cv::GrayLoader grayloader;
    fbow::createORBvocabulary(voc, system.mpTracker->mpExtractor0.get(), grayloader, std::get<1>(storage));
    voc.saveToFile(vocPath);
  }

  system.run();
  system.mThreads["tracker"]->join();
  system.stop();
  return 0;
}
