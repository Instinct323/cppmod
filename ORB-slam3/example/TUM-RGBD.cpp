#include <boost/format.hpp>
#include <filesystem>

#include "utils/cv.hpp"
#include "utils/indicators.hpp"
#include "utils/glog.hpp"
#include "zjcv/dataset/tum_rgbd.hpp"
#include "zjcv/imu.hpp"
#include "zjcv/slam.hpp"

std::tuple<
    dataset::Timestamps, dataset::Filenames,
    dataset::Timestamps, dataset::Filenames,
    dataset::Timestamps, dataset::Poses> storage;


// Tracking
void slam::Tracker::run() {
  glog::Timer timer;
  cv::GrayLoader grayloader;
  auto pbar = indicators::getProgressBar(std::get<0>(storage).size());

  while (!pbar.is_completed()) {
    int i = pbar.current();
    cv::Mat img = grayloader(std::get<1>(storage)[i]), imgDepth = cv::imread(std::get<3>(storage)[i], -1);
    imgDepth.convertTo(imgDepth, CV_32F, mDepthInvScale);

    // 读入数据
    timer.reset();
    grab_image(std::get<0>(storage)[i], img, imgDepth);

    mpSystem->set_desc("track-cost", (boost::format("%.1fms") % (1e3 * timer.elapsed())).str());
    mpSystem->set_desc("state", mState);
    indicators::set_desc(pbar, mpSystem->get_desc(), false);
    pbar.tick();
  }

  // 写入位姿
  LOG(INFO) << "Wait for writing pose...";
  mpSystem->mpAtlas->export_poses(boost::format("pose-%d.txt"));
}


int main(int argc, char **argv) {
  putenv("DISPLAY=host.docker.internal:0");
  glog::Logger logger(argv);
  LOG(INFO) << "Thread pool size: " << parallel::thread_pool_size;

  // config
  YAML::Node cfg = YAML::LoadFile("/home/workbench/cppmod/ORB-slam3/cfg/TUM-RGBD.yaml");
  slam::System system(cfg);
  system.mpTracker->reload(cfg["tracker"]);

  // 载入并校验数据
  dataset::TumRGBD tum_rgbd(cfg["dataset"].as<std::string>());
  tum_rgbd.load_image(std::get<0>(storage), std::get<1>(storage), "rgb.txt");
  tum_rgbd.load_image(std::get<2>(storage), std::get<3>(storage), "depth.txt");
  tum_rgbd.load_pose(std::get<4>(storage), std::get<5>(storage));

  assert(std::get<0>(storage).size() == std::get<2>(storage).size());
  LOG(INFO) << "Images: " << std::get<0>(storage).size()
            << ", Poses: " << std::get<4>(storage).size();

  system.run();
  system.mThreads["tracker"]->join();
  system.stop();
  return 0;
}
