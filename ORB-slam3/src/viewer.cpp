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

  glLineWidth(1);
  pangolin::Var<Vec3> pos("ui.P", Vec3());
  while (mpSystem->mbRunning) {
    timer.reset();
    feature::Frame::Ptr pFrame = pTracker->mpLastFrame;
    if (pFrame) {
      auto &cur_map = mpSystem->mpAtlas->mpCurMap;
      // ----- OpenCV -----
      auto thread1 = parallel::thread_pool.emplace(2, &feature::Frame::draw, pFrame.get());
      // ----- Pangolin -----
      const Sophus::SE3f &T_imu_world = pFrame->mPose.T_imu_world;
      pos = Vec3{T_imu_world.translation()};
      pgl_fig->clear();
      pangolin::OpenGlMatrix Tiw(T_imu_world.matrix());
      pgl_fig->follow(Tiw);
      // 绘制地图
      cur_map->draw(pFrame);
      pgl_fig->draw();
      thread1->join();
    }
    // 检查系统状态
    if (!pgl_fig->is_running()) mpSystem->mbRunning = false;
    int cost = static_cast<int>(timer.count() * 1e3);
    mpSystem->set_desc("view-FPS", std::to_string(1000 / std::max(delay, cost)));
    cv::waitKey(std::max(1, delay - cost));
  }
}

}
