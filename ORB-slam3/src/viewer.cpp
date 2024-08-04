#include <boost/format.hpp>

#include "utils/glog.hpp"
#include "zjcv/dataset/base.hpp"
#include "zjcv/slam.hpp"


extern std::tuple<
    dataset::Timestamps, dataset::Filenames,
    dataset::Timestamps, dataset::Filenames,
    dataset::Timestamps, dataset::IMUsamples,
    dataset::Timestamps, dataset::Poses> storage;


namespace slam {


void Viewer::run() {
  YAML::Node cfg = mpSystem->mCfg["viewer"];

  glog::Timer timer;
  Tracker::Ptr pTracker = mpSystem->mpTracker;
  pangolin::Figure::Ptr pgl_fig = pangolin::Figure::from_yaml(cfg);

  while (mpSystem->mbRunning) {
    timer.reset();
    Frame::Ptr pFrame = pTracker->mpLastFrame;
    if (pFrame) {
      // ----- Pangolin -----
      pgl_fig->clear();
      pangolin::OpenGlMatrix T_world_imu(pFrame->mPose.T_world_imu.matrix());
      pgl_fig->follow(T_world_imu);
      // 绘制当前帧
      pFrame->show_in_pangolin(imu_size, mp_size, lead_color.data(), mp_color.data());
      // 绘制地图
      mpSystem->mpAtlas->mpCurMap->draw();
      pgl_fig->draw();
      // ----- OpenCV -----
      pFrame->draw();
    }
    // 检查系统状态
    if (!pgl_fig->is_running()) mpSystem->mbRunning = false;
    int cost = static_cast<int>(timer.count() * 1e3);
    mpSystem->set_desc("view-FPS", std::to_string(1000 / std::max(delay, cost)));
    cv::waitKey(std::max(1, delay - cost));
  }
}

}
