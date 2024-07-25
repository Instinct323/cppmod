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
) : mpSystem(pSystem), MIN_MATCHES(cfg["min_matches"].as<int>()), MAX_MATCHES(cfg["max_matches"].as<int>()),
    mpCam0(camera::from_yaml(cfg["cam0"])), mpCam1(camera::from_yaml(cfg["cam1"])),
    mpIMU(IMU::Device::from_yaml(cfg["imu"])) {
  ASSERT(!is_monocular() || !is_inertial(), "Not implemented")
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
  if (!mpIMUpreint) {
    ASSERT(is_inertial(), "Not inertial tracker")
    mpIMUpreint = std::make_shared<IMU::Preintegration>(mpIMU.get(), vTimestamp.empty() ? tCurframe : vTimestamp[0]);
  }
  mpIMUpreint->integrate(tCurframe, vTimestamp, vSample);
}


// 基于特征点
namespace feature {


// Frame
Frame::Frame(System *pSystem, const double &timestamp,
             const cv::Mat &img0, const cv::Mat &img1
) : mpSystem(pSystem), mTimestamp(timestamp), mImg0(img0), mImg1(img1) {}


Frame::~Frame() {
  for (auto &m: mvpMappts) if (m) m->erase_obs(this);
}


int Frame::init_mappoints(const std::vector<cv::DMatch> &matches) {
  int n = mvKps0.size();
  mvpMappts = std::vector<std::shared_ptr<Mappoint>>(n, nullptr);

  if (!matches.empty()) {
    Sophus::SE3f &T_cam0_cam1 = mpSystem->mpTracker->T_cam0_cam1;
    const camera::Base::Ptr &pCam0 = mpSystem->mpTracker->mpCam0, &pCam1 = mpSystem->mpTracker->mpCam1;

    bool is_fisheye = pCam0->get_type() == camera::KANNALA_BRANDT;
    for (auto m: matches) {
      int i = m.queryIdx, j = m.trainIdx;
      Eigen::Vector3f P0_cam0 = pCam0->unproject(mvKps0[i].pt),
          P1_cam1 = pCam1->unproject(mvKps1[j].pt);
      // 鱼眼相机余弦判断
      if (is_fisheye) {
        Eigen::Vector3f P1_cam0 = T_cam0_cam1 * P1_cam1;
        if (Eigen::cos(P0_cam0, P1_cam0) > 0.9998) continue;
      }
      // 创建地图点
      mvpMappts[i] = std::make_shared<Mappoint>();
      mvpMappts[i]->add_obs(this, i);
      mvpMappts[i]->add_obs(this, n + j);
    }
  }
  return n;
}


int Frame::connect_frame(Frame::Ptr &other, const std::vector<cv::DMatch> &matches) {
  int n = std::min(int(matches.size()), mpSystem->mpTracker->MAX_MATCHES);
  for (int i = 0; i < n; ++i) {
    const cv::DMatch &m = matches[i];
    auto mpit_cur = mvpMappts.begin() + m.queryIdx;
    auto mpit_ref = other->mvpMappts.begin() + m.trainIdx;
    // 合并: 当前帧 -> 参考帧
    if (*mpit_ref) {
      if (*mpit_cur) {
        **mpit_ref += **mpit_cur;
      } else {
        (*mpit_ref)->add_obs(this, m.queryIdx);
      }
      *mpit_cur = *mpit_ref;
    } else {
      // 合并: 参考帧 -> 当前帧
      if (!*mpit_cur) {
        *mpit_cur = std::make_shared<Mappoint>();
        (*mpit_cur)->add_obs(this, m.queryIdx);
      }
      (*mpit_cur)->add_obs(other.get(), m.trainIdx);
      *mpit_ref = *mpit_cur;
    }
  }
  return n;
}


// Mappoint
void Mappoint::add_obs(Frame *pFrame, const int &idx) { mObs.emplace_back(pFrame, idx); }


void Mappoint::erase_obs(Frame *pFrame) {
  mObs.erase(std::remove_if(
      mObs.begin(), mObs.end(),
      [pFrame](const Observation &obs) { return obs.first == pFrame; }), mObs.end());
}


Mappoint &Mappoint::operator+=(const Mappoint &other) {
  mObs.insert(mObs.end(), other.mObs.begin(), other.mObs.end());
  return *this;
}

}

}
