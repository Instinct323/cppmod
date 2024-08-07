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


struct Vec3 {
    Eigen::Vector3f p = Eigen::Vector3f::Zero();
};


std::ostream &operator<<(std::ostream &os, const Vec3 &p) {
  return os << p.p[0] << " " << p.p[1] << " " << p.p[2];
}


std::istream &operator>>(std::istream &os, const Vec3 &p) {
  return os;
}


void Viewer::run() {
  YAML::Node cfg = mpSystem->mCfg["viewer"];

  glog::Timer timer;
  Tracker::Ptr pTracker = mpSystem->mpTracker;
  pangolin::Figure::Ptr pgl_fig = pangolin::Figure::from_yaml(cfg);

  // 创建面板
  pangolin::Var<Vec3> pos("ui.P", Vec3());
  while (mpSystem->mbRunning) {
    timer.reset();
    feature::Frame::Ptr pFrame = pTracker->mpLastFrame;
    if (pFrame) {
      auto &cur_map = mpSystem->mpAtlas->mpCurMap;
      if (pTracker->mState == TrackState::OK) {
        const Sophus::SE3f &T_imu_world = pFrame->mPose.T_imu_world;
        pos = Vec3{T_imu_world.translation()};
        // ----- Pangolin -----
        pgl_fig->clear();
        pangolin::OpenGlMatrix Tiw(T_imu_world.matrix());
        pgl_fig->follow(Tiw);
        // 绘制当前帧
        pFrame->show_in_opengl(imu_size, lead_color.data(), true);
        {
          parallel::ScopedLock lock(cur_map->apKeyFrames.mutex);
          if (cur_map->apKeyFrames->size()) {
            glColor3fv(trail_color.data());
            glBegin(GL_LINES);
            glVertex3fv(pFrame->get_pos().data());
            glVertex3fv(cur_map->apKeyFrames->back()->get_pos().data());
            glEnd();
          }
        }
        // 绘制地图
        cur_map->draw();
        pgl_fig->draw();
      }
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
