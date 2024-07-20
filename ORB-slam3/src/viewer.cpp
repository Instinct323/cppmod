#include <boost/format.hpp>

#include "utils/glog.hpp"
#include "zjcv/slam.hpp"

namespace slam {


Viewer::Viewer(System *pSystem, const YAML::Node &cfg
) : mpSystem(pSystem) {
  YAML::Node cfgViewer = cfg["viewer"];
  int fps = cfgViewer["fps"].as<int>();
  ASSERT(fps > 0, "Viewer: The delay must be greater than 0")
  delay = 1000 / fps;
  sample_stride = cfgViewer["sample_stride"].as<int>();
  trail_size = cfgViewer["trail_size"].as<int>();
  imu_size = cfgViewer["imu_size"].as<float>();
  lead_color = YAML::toEigen<float>(cfgViewer["lead_color"]);
  trail_color = YAML::toEigen<float>(cfgViewer["trail_color"]);
}


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
      glColor3f(lead_color[0], lead_color[1], lead_color[2]);
      pangolin::draw_imu(T_world_imu, imu_size);
      // 绘制关键帧 (末 100)
      glColor3f(trail_color[0], trail_color[1], trail_color[2]);
      std::vector<Frame::Ptr> &mvpKf = mpSystem->mpAtlas->mpCurMap->mvpKeyFrames;
      for (int i = MAX(0, int(mvpKf.size()) - trail_size * sample_stride);
           i < mvpKf.size(); i += sample_stride) {
        Frame::Ptr pKf = mvpKf[i];
        if (pKf) {
          pangolin::OpenGlMatrix T_wi(pKf->mPose.T_world_imu.matrix());
          pangolin::draw_imu(T_wi, imu_size);
        }
      }
      pgl_fig->draw();
      // ----- OpenCV -----
      pFrame->draw();
    }
    // 检查系统状态
    if (!pgl_fig->is_running()) mpSystem->mbRunning = false;
    int cost = static_cast<int>(timer.count() * 1e3);
    mpSystem->set_desc("view-FPS", std::to_string(1000 / MAX(delay, cost)));
    cv::waitKey(MAX(1, delay - cost));
  }
}

}
