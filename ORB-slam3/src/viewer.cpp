#include <boost/format.hpp>

#include "frame.hpp"
#include "tracker.hpp"
#include "utils/glog.hpp"
#include "viewer.hpp"

namespace slam {

void Viewer::run() {
  YAML::Node cfg = mpSystem->mCfg["viewer"];
  auto imu_size = cfg["imu_size"].as<float>();
  auto queue_size = cfg["queue_size"].as<int>();

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
      glColor3f(0.0, 0.0, 1.0);
      pangolin::draw_imu(T_world_imu, imu_size);
      // 绘制关键帧 (末 100)
      glColor3f(0.0, 1.0, 0.0);
      std::vector<Frame::Ptr> &mvpKf = mpSystem->mpAtlas->mpCurMap->mvpKeyFrames;
      for (int i = MAX(0, int(mvpKf.size()) - queue_size); i < mvpKf.size(); ++i) {
        Frame::Ptr pKf = mvpKf[i];
        if (pKf) {
          pangolin::OpenGlMatrix T_wi(pKf->mPose.T_world_imu.matrix());
          pangolin::draw_imu(T_wi, imu_size);
        }
      }
      pgl_fig->draw();
      // ----- OpenCV -----
      cv::Mat img0 = pFrame->mImg0.clone(), img1 = pFrame->mImg1.clone(), toshow;
      // Image 0
      pTracker->mpCam0->undistort(img0, img0);
      cv::cvtColor(img0, img0, cv::COLOR_GRAY2BGR);
      if (img1.empty()) {
        cv::drawKeypoints(img0, pFrame->mvKps0, toshow);
      } else {
        // Image 1
        pTracker->mpCam1->undistort(img1, img1);
        cv::cvtColor(img1, img1, cv::COLOR_GRAY2BGR);
        cv::drawMatches(img0, pFrame->mvKps0, img1, pFrame->mvKps1, pFrame->mStereoMatches, toshow);
      }
      cv::imshow("Image", toshow);
    }
    // 检查系统状态
    if (!pgl_fig->is_running()) mpSystem->mbRunning = false;
    int cost = static_cast<int>(timer.count() * 1e3);
    mpSystem->set_desc("view-FPS", std::to_string(1000 / MAX(mDelay, cost)));
    cv::waitKey(MAX(1, mDelay - cost));
  }
}

}
