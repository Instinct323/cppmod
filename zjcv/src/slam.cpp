#include "zjcv/slam.hpp"

namespace slam {

using namespace feature;


// Atlas
Map::Ptr Atlas::create_map() {
  mpCurMap = std::make_shared<Map>(mpSystem);
  mvpMaps.push_back(mpCurMap);
  return mpCurMap;
}


// System
System::System(const YAML::Node &cfg
) : mCfg(cfg), mpAtlas(new Atlas(this, cfg["atlas"])), mpTracker(new Tracker(this, cfg["tracker"])),
    mpViewer(new Viewer(this, cfg["viewer"])) {}


void System::run() {
  mbRunning = true;
  mThreads["tracker"] = parallel::thread_pool.emplace(0, &Tracker::run, mpTracker);
  mThreads["viewer"] = parallel::thread_pool.emplace(0, &Viewer::run, mpViewer);
}


void System::stop() {
  mbRunning = false;
  parallel::thread_pool.join();
}


// Tracker
Tracker::Tracker(System *pSystem, const YAML::Node &cfg
) : mpSystem(pSystem), mpIMU(IMU::Device::from_yaml(cfg["imu"])),
    mpCam0(camera::from_yaml(cfg["cam0"])), mpCam1(camera::from_yaml(cfg["cam1"])) {
  ASSERT(!is_monocular(), "Not implemented")
  ASSERT(mpCam0, "Camera0 not found")
  if (is_stereo()) {
    // 校对相机类型
    ASSERT(mpCam0->get_type() == mpCam1->get_type(), "Camera0 and Camera1 must be the same type")
    T_cam0_cam1 = mpCam0->T_cam_imu * mpCam1->T_cam_imu.inverse();
  }
}


void Tracker::grab_image(const double &timestamp, const cv::Mat &img0, const cv::Mat &img1) {
  ASSERT(!is_inertial() || timestamp <= mpIMUpreint->mtEnd, "Grab image before IMU data is integrated")
  ASSERT(is_monocular() == img1.empty(), "Invalid image pair")
  mpLastFrame = mpCurFrame;
  mpCurFrame = std::make_shared<Frame>(mpSystem, timestamp, img0, img1);
  // 如果是第一帧, 补充位姿
  if (!mpLastFrame) mpCurFrame->mPose.set_zero();
  mpCurFrame->process();
}


void Tracker::grab_imu(const double &tCurframe, const std::vector<double> &vTimestamp, const std::vector<IMU::Sample> &vSample) {
  if (!mpIMUpreint) mpIMUpreint = std::make_shared<IMU::Preintegration>(mpIMU.get(), vTimestamp.empty() ? tCurframe : vTimestamp[0]);
  mpIMUpreint->integrate(tCurframe, vTimestamp, vSample);
}


// 基于特征点
namespace feature {


// Frame
Frame::Frame(System *pSystem, const double &timestamp,
             const cv::Mat &img0, const cv::Mat &img1
) : mpSystem(pSystem), mTimestamp(timestamp), mImg0(img0), mImg1(img1) {}


// Mappoint
void Mappoint::add_obs(Frame *pFrame, const int &idx) { mObs.emplace_back(pFrame, idx); }


void Mappoint::erase_obs(Frame *pFrame) {
  mObs.erase(std::remove_if(
      mObs.begin(), mObs.end(),
      [pFrame](const Observation &obs) { return obs.first == pFrame; }), mObs.end());
}

}

}
