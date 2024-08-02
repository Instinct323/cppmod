#include <g2o/solvers/dense/linear_solver_dense.h>

#include "utils/sophus.hpp"
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
  ASSERT(parallel::thread_pool_size > 2, "The hardware concurrency is insufficient, please adjust the thread limit manually")
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
) : mpSystem(pSystem),
    MIN_MATCHES(cfg["min_matches"].as<int>()), MAX_MATCHES(cfg["max_matches"].as<int>()),
    KEY_MATCHES(cfg["key_matches"].as<int>()), LOST_TIMEOUT(cfg["lost_timeout"].as<double>()),

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


void Tracker::switch_state(TrackState state) {
  mState = state;
  if (mState == TrackState::RECENTLY_LOST) {
    if (!mpRefFrame || !mpCurFrame || (mpCurFrame->mTimestamp - mpRefFrame->mTimestamp > LOST_TIMEOUT)) mState = TrackState::LOST;
  }
  if (mState == TrackState::LOST) {
    mpLastFrame = nullptr;
    mpCurFrame = nullptr;
    mpRefFrame = nullptr;
    // todo: 切换地图
  }
}


// 基于特征点
namespace feature {


// Frame
size_t Frame::FRAME_COUNT = 0;
size_t Frame::KEY_COUNT = 0;


Frame::Frame(System *pSystem, const double &timestamp,
             const cv::Mat &img0, const cv::Mat &img1
) : mpSystem(pSystem), mId(++FRAME_COUNT), mTimestamp(timestamp), mImg0(img0), mImg1(img1),
    mJoint(&mPose.T_imu_world) {}


int Frame::stereo_triangulation(const Frame::Ptr &shared_this, const std::vector<cv::DMatch> &stereo_matches) {
  int cnt = 0;
  if (!stereo_matches.empty()) {
    Sophus::SE3f &T_cam0_cam1 = mpSystem->mpTracker->T_cam0_cam1;
    for (auto m: stereo_matches) {
      int i = m.queryIdx, j = m.trainIdx;
      // 余弦判断
      const Eigen::Vector3f &P0_cam0 = mvUnprojs0[i], &P1_cam1 = mvUnprojs1[j];
      Eigen::Vector3f P1_cam0 = T_cam0_cam1 * P1_cam1;
      if (Eigen::cos(P0_cam0, P1_cam0) > 0.9998) continue;
      cnt += 1;
      // 创建地图点
      if (!mvpMappts[i]) {
        mvpMappts[i] = mpSystem->get_cur_map()->create_mappoint();
        mvpMappts[i]->add_obs(shared_this, i);
      }
      mvpMappts[i]->add_obs(shared_this, j, true);
      mvpMappts[i]->triangulation();
    }
  }
  return cnt;
}


int Frame::connect_frame(Frame::Ptr &shared_this, Frame::Ptr &other, std::vector<cv::DMatch> &matches) {
  // matches: 参考帧 -> 当前帧
  int n = std::min(int(matches.size()), mpSystem->mpTracker->MAX_MATCHES);
  for (int i = 0; i < n; ++i) {
    const cv::DMatch &m = matches[i];
    auto mpit_ref = other->mvpMappts.begin() + m.queryIdx;
    auto mpit_cur = mvpMappts.begin() + m.trainIdx;
    // 合并: 当前帧 -> 参考帧
    if (*mpit_ref) {
      if (*mpit_cur) {
        **mpit_ref += **mpit_cur;
      } else {
        (*mpit_ref)->add_obs(shared_this, m.trainIdx);
      }
      *mpit_cur = *mpit_ref;
    } else {
      // 合并: 参考帧 -> 当前帧
      if (!*mpit_cur) {
        *mpit_cur = mpSystem->get_cur_map()->create_mappoint();
        (*mpit_cur)->add_obs(shared_this, m.trainIdx);
      }
      (*mpit_cur)->add_obs(other, m.queryIdx);
      *mpit_ref = *mpit_cur;
    }
  }
  return n;
}


void Frame::mark_keyframe() {
  mIdKey = ++KEY_COUNT;
}


void Frame::prune() {
  for (Mappoint::Ptr &m: mvpMappts) if (m) m->prune();
}


// Map
Mappoint::Ptr Map::create_mappoint() {
  auto pMappt = std::make_shared<Mappoint>(mpSystem);
  mvpTmpMappts.push_back(pMappt);
  return pMappt;
}


void Map::insert_keyframe(const std::shared_ptr<Frame> &pKF) {
  pKF->mark_keyframe();
  mvpKeyFrames.push_back(pKF);
  // 整理临时地图点
  mvpKeyFrames.reserve(mvpKeyFrames.size() + mvpTmpMappts.size());
  for (auto &pMappt: mvpTmpMappts) {
    if (pMappt.expired()) continue;
    mvpMappts.push_back(pMappt);
  }
}


// Mappoint
int Mappoint::frame_count() {
  prune();
  std::set<Frame *> sFrames;
  parallel::ScopedLock lock(mMutexObs);
  for (auto &obs: mObs) sFrames.insert(obs.first.lock().get());
  return sFrames.size();
}


void Mappoint::prune() {
  parallel::ScopedLock lock(mMutexObs);
  auto begit = std::remove_if(
      mObs.begin(), mObs.end(),
      [](const Observation &obs) { return obs.first.expired(); });
  mObs.erase(begit, mObs.end());
}


void Mappoint::add_obs(const Frame::Ptr &pFrame, const int &idx, bool is_right) {
  parallel::ScopedLock lock(mMutexObs);
  mObs.emplace_back(pFrame, is_right ? (idx + pFrame->mvUnprojs0.size()) : idx);
  // 如果是右相机观测, 调整位置
  if (is_right) {
    size_t fid = pFrame->mId;
    auto it = std::find_if(
        mObs.begin(), mObs.end(),
        [&fid](const Observation &obs) { return !obs.first.expired() && obs.first.lock()->mId == fid; });
    std::swap(*(it + 1), mObs.back());
  }
}


void Mappoint::clear() {
  parallel::ScopedLock lock(mMutexObs);
  for (auto [wpf, i]: mObs) {
    if (wpf.expired()) continue;
    Frame::Ptr spf = wpf.lock();
    if (i < spf->mvUnprojs0.size()) spf->mvpMappts[i] = nullptr;
  }
  mObs.clear();
}


Mappoint &Mappoint::operator+=(const Mappoint &other) {
  parallel::ScopedLock lock(mMutexObs);
  mObs.insert(mObs.end(), other.mObs.begin(), other.mObs.end());
  return *this;
}


void Mappoint::triangulation() {
  const camera::Base::Ptr &pCam0 = mpSystem->mpTracker->mpCam0, &pCam1 = mpSystem->mpTracker->mpCam1;
  std::vector<Eigen::Vector3f> vP_cam;
  std::vector<Sophus::SE3f> vT_cam_ref;

  prune();
  {
    parallel::ScopedLock lock(mMutexObs);
    for (auto &obs: mObs) {
      if (obs.first.expired()) continue;
      Frame::Ptr pFrame = obs.first.lock();
      int i = obs.second;
      int n = pFrame->mvUnprojs0.size();

      bool is_left = i < n;
      const camera::Base::Ptr &pCam = is_left ? pCam0 : pCam1;

      vP_cam.push_back(is_left ? pFrame->mvUnprojs0[i] : pFrame->mvUnprojs1[i - n]);
      vT_cam_ref.push_back(pCam->T_cam_imu * pFrame->mPose.T_imu_world);
    }
  }
  mReprErr = Sophus::triangulation(vP_cam, vT_cam_ref, mPos);
}


// optimize
void optimize_pose(Frame::Ptr pFrame) {
  std::vector<std::shared_ptr<Frame>> vpFrames = {pFrame};
  bundle_adjustment<g2o::LinearSolverDense>(vpFrames, pFrame->mpSystem->get_cur_map()->mvpTmpMappts, true);
}


template<template<typename> class LinearSolverTp>
void bundle_adjustment(std::vector<std::shared_ptr<Frame>> &vpFrames,
                       std::vector<std::weak_ptr<Mappoint>> &vpMappts,
                       bool only_pose) {
  g2o::Optimizer<6, 3, LinearSolverTp, g2o::OptimizationAlgorithmLevenberg> optimizer;
  int id_vex = 0;

  // Vertex: 帧位姿
  for (int &i = id_vex; i < vpFrames.size(); ++i) {
    vpFrames[i]->mIdVex = i;
    auto *vSE3 = new g2o::VertexSE3Expmap;
    vSE3->setEstimate(Sophus::toG2O(vpFrames[i]->mPose.T_imu_world));
    vSE3->setId(i);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);
  }
}

}

}
