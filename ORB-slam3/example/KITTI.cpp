#include <boost/format.hpp>
#include <filesystem>

#include "utils/cv.hpp"
#include "utils/indicators.hpp"
#include "utils/glog.hpp"
#include "zjcv/dataset/kitti.hpp"
#include "zjcv/imu.hpp"
#include "zjcv/slam.hpp"

std::tuple<dataset::Timestamps, dataset::Filenames, dataset::Filenames> storage;


// Tracking
void slam::Tracker::run() {
  glog::Timer timer;
  cv::GrayLoader grayloader;
  auto pbar = indicators::getProgressBar(std::get<0>(storage).size());

  while (!pbar.is_completed()) {
    int i = pbar.current();
    cv::Mat imgLeft = grayloader(std::get<1>(storage)[i]), imgRight = grayloader(std::get<2>(storage)[i]);

    // 读入数据
    timer.reset();
    grab_image(std::get<0>(storage)[i], imgLeft, imgRight);

    mpSystem->set_desc("track-cost", (boost::format("%.1fms") % (1e3 * timer.elapsed())).str());
    mpSystem->set_desc("state", mState);
    indicators::set_desc(pbar, mpSystem->get_desc(), false);
    pbar.tick();
  }

  LOG(INFO) << "Wait for writing pose...";
  mpSystem->mpAtlas->export_poses(boost::format("pose-%d.txt"));
}


int main(int argc, char **argv) {
  glog::Logger logger(argv);
  LOG(INFO) << "Thread pool size: " << parallel::thread_pool_size;

  // config
  YAML::Node cfg = YAML::LoadFile("/home/workbench/cppmod/ORB-slam3/cfg/KITTI.yaml");
  slam::System system(cfg);
  system.mpTracker->reload(cfg["tracker"]);

  // 载入并校验数据
  dataset::Kitti kitti(cfg["dataset"].as<std::string>(), cfg["seq_id"].as<int>());
  kitti.load_timestamp(std::get<0>(storage));
  kitti.load_image(std::get<1>(storage), "image_0");
  kitti.load_image(std::get<2>(storage), "image_1");

  assert(std::get<0>(storage).size() == std::get<1>(storage).size() &&
         std::get<0>(storage).size() == std::get<2>(storage).size());
  LOG(INFO) << "Timestamps: " << std::get<0>(storage).size()
            << ", Image0: " << std::get<1>(storage).size()
            << ", Image1: " << std::get<2>(storage).size();
  // LOG(INFO) << "P0:" << std::endl << kitti.load_calib("P0");
  // LOG(INFO) << "P1:" << std::endl << kitti.load_calib("P1");

  system.run();
  system.mThreads["tracker"]->join();
  system.stop();
  return 0;
}
