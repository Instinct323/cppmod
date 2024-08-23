#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include "utils/pangolin.hpp"
#include "utils/sophus.hpp"
#include "zjcv/slam.hpp"

namespace slam {


// Atlas
feature::Map::Ptr Atlas::create_map() {
  mpCurMap = std::make_shared<feature::Map>(mpSystem);
  mvpMaps.push_back(mpCurMap);
  return mpCurMap;
}


void Atlas::export_poses(boost::format fmt) const {
  for (int i = 0; i < mvpMaps.size(); ++i) {
    const auto &pMap = mvpMaps[i];
    std::string filename = (fmt % i).str();
    std::ofstream ofs(filename);
    // 写入位姿
    for (const auto &pKF: *pMap->apKeyFrames)
      ofs << std::fixed << size_t(pKF->mTimestamp * 1e9) << " " << pKF->mPose.T_world_imu << std::endl;
    ofs.close();
  }
}


void Atlas::run() {
  while (mpSystem->mbRunning) {
    int n_kf = mpCurMap->apKeyFrames->size();
    mpSystem->set_desc("n-kf", n_kf);
    int raw_kf = n_kf - mpCurMap->miKeyFrame;
    mpSystem->set_desc("raw-kf", raw_kf);

    if (!mbSleep) {
      feature::Map::Ptr cur_map = mpCurMap;
      if (raw_kf >= 3) {
        cur_map->local_mapping();
      }
      usleep(50000);
    }

    if (mbSleep) {
      mbSleep = false;
      sleep(4);
    }
  }
}


// System
System::System(const YAML::Node &cfg
) : mCfg(cfg), mpAtlas(new Atlas(this, cfg["atlas"])), mpTracker(new Tracker(this, cfg["tracker"])),
    mpViewer(new Viewer(this, cfg["viewer"])) {}


void System::run() {
  assert(parallel::thread_pool_size > 2 && "The hardware concurrency is insufficient, please adjust the thread limit manually");
  mbRunning = true;
  mThreads["tracker"] = parallel::thread_pool.emplace(0, &Tracker::run, mpTracker);
  mThreads["viewer"] = parallel::thread_pool.emplace(0, &Viewer::run, mpViewer);
  // mThreads["atlas"] = parallel::thread_pool.emplace(0, &Atlas::run, mpAtlas);
}


void System::stop() {
  mbRunning = false;
  parallel::thread_pool.join(0);
  mThreads.clear();
}


// Tracker
Tracker::Tracker(System *pSystem, const YAML::Node &cfg
) : mpSystem(pSystem),
    MIN_MATCHES(cfg["min_matches"].as<int>()), MAX_MATCHES(cfg["max_matches"].as<int>()),
    KEY_MATCHES_RADIO(cfg["key_matches_radio"].as<float>()), LOST_TIMEOUT(cfg["lost_timeout"].as<double>()),

    mpCam0(camera::from_yaml(cfg["cam0"])), mpCam1(camera::from_yaml(cfg["cam1"])),
    mDepthInvScale(1 / cfg["depth_scale"].as<float>(-1)), mpIMU(IMU::Device::from_yaml(cfg["imu"])) {

  assert(mpCam0 && "Camera0 not found");
  LOG(INFO) << "Tracker: " << "Camera0 ready";
  // 双目模式校对
  if (is_stereo()) {
    assert(mpCam0->get_type() == mpCam1->get_type() && "Camera0 and Camera1 must be the same type");
    LOG(INFO) << "Tracker: " << "Camera1 ready";
  }
  // 深度相机模式校对
  if (is_depth()) {
    assert(is_monocular() && "Depth camera must be monocular");
    LOG(INFO) << "Tracker: " << "Depth camera ready";
  }
  // IMU 模式校对
  if (is_inertial()) {
    LOG(FATAL) << "Not implemented";
    LOG(INFO) << "Tracker: " << "IMU ready";
  }
}


void Tracker::grab_image(const double &timestamp, const cv::Mat &img0, const cv::Mat &img1) {
  assert(!is_inertial() || timestamp <= mpIMUpreint->mtEnd && "Grab image before IMU data is integrated");
  // assert(is_monocular() == img1.empty() && "Invalid image pair");

  parallel::ScopedLock lock(mpSystem->get_cur_map()->mPosesMutex);
  mpLastFrame = mpCurFrame;
  if (mpLastFrame) mpLastFrame->update_pose();
  mpCurFrame = std::make_shared<feature::Frame>(mpSystem, timestamp, img0, img1);
  mpCurFrame->process();
}


void Tracker::grab_imu(const double &tCurframe, const std::vector<double> &vTimestamp, const std::vector<IMU::Sample> &vSample) {
  if (!mpIMUpreint) {
    assert(is_inertial() && "Not inertial tracker");
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
    mpSystem->mpAtlas->create_map();
  }
}


// Viewer
Viewer::Viewer(System *pSystem, const YAML::Node &cfg
) : mpSystem(pSystem), delay(1000 / cfg["fps"].as<int>()),
    imu_size(cfg["imu_size"].as<float>()), mp_size(cfg["mp_size"].as<float>()), trail_size(cfg["trail_size"].as<size_t>()),
    lead_color(YAML::toEigen<float>(cfg["lead_color"])), trail_color(YAML::toEigen<float>(cfg["trail_color"])),
    mp_color(YAML::toEigen<float>(cfg["mp_color"])) {
  assert(delay > 0 && "Viewer: The delay must be greater than 0");
}


// 基于特征点
namespace feature {


// Frame
bool Frame::mbNegDepth = true;
size_t Frame::FRAME_COUNT = 0;
size_t Frame::KEY_COUNT = 0;


Frame::Frame(System *pSystem, const double &timestamp,
             const cv::Mat &img0, const cv::Mat &img1
) : mpSystem(pSystem), mId(++FRAME_COUNT), mTimestamp(timestamp), mImg0(img0), mImg1(img1),
    mpRefFrame(pSystem->mpTracker->mpRefFrame), mJoint(&mPose.T_world_imu) {}


int Frame::rgbd_init(const Frame::Ptr &shared_this) {
  assert(mmpMappts.empty() && "The map points have been initialized");
  int cnt = 0;
  const Sophus::SE3f T_cam_world = mPose.T_imu_world * mpSystem->mpTracker->mpCam0->T_cam_imu;

  float depth;
  for (int i = 0; i < mvUnprojs0.size(); ++i) {
    if (get_depth(i, depth)) {
      Eigen::Vector3f P_cam = mvUnprojs0[i];
      P_cam[2] = 1;
      P_cam *= depth;

      cnt++;
      auto it = mmpMappts.insert({i, mpSystem->get_cur_map()->create_mappoint()}).first;
      it->second->add_obs(shared_this, i);
      it->second->set_pos(T_cam_world * P_cam);
    }
  }
  return cnt;
}


int Frame::stereo_triangulation(const Frame::Ptr &shared_this, const std::vector<cv::DMatch> &left2right) {
  std::atomic_int cnt = 0;
  if (!left2right.empty()) {
    const Sophus::SE3f &T_cam0_cam1 = mpSystem->mpTracker->mpCam1->T_cam_imu.inverse() * mpSystem->mpTracker->mpCam0->T_cam_imu,
        Identity,
        T_cam0_world = mPose.T_imu_world * mpSystem->mpTracker->mpCam0->T_cam_imu;
    // 三角剖分, 生成地图点
    auto task = [this, &shared_this, &cnt, &Identity, &T_cam0_cam1, &T_cam0_world
    ](int i, const Eigen::Vector3f &P0_cam0, const Eigen::Vector3f &P1_cam1) {
        Eigen::Vector3f P_cam0;
        float rep_error = Sophus::triangulation({P0_cam0, P1_cam1}, {Identity, T_cam0_cam1}, P_cam0);
        if (rep_error >= 0) {
          cnt += 1;
          mvUnprojs0[i][2] = mbNegDepth ? -P_cam0[2] : P_cam0[2];
          // 创建地图点
          auto it = mmpMappts.find(i);
          if (it == mmpMappts.end()) {
            it = mmpMappts.insert({i, mpSystem->get_cur_map()->create_mappoint()}).first;
            it->second->add_obs(shared_this, i);
          }
          it->second->set_pos(T_cam0_world * P_cam0);
        }
    };
    for (auto m: left2right) {
      int i = m.queryIdx, j = m.trainIdx;
      // 余弦判断
      const Eigen::Vector3f &P0_cam0 = mvUnprojs0[i], &P1_cam1 = mvUnprojs1[j];
      Eigen::Vector3f P1_cam0 = T_cam0_cam1 * P0_cam0;
      if (Eigen::cos(P0_cam0, P1_cam0) > STEREO_OBS_COS_THRESH) continue;
      // 三角剖分线程
      parallel::thread_pool.emplace(1, task, i, P0_cam0, P1_cam1);
    }
    parallel::thread_pool.join(1);
  }
  return cnt;
}


int Frame::connect_frame(Frame::Ptr &shared_this, Frame::Ptr &ref, std::vector<cv::DMatch> &ref2this) {
  // matches: 参考帧 -> 当前帧
  int n = std::min(int(ref2this.size()), mpSystem->mpTracker->MAX_MATCHES);
  for (int i = 0; i < n; ++i) {
    const cv::DMatch &m = ref2this[i];
    auto mpit_ref = ref->mmpMappts.find(m.queryIdx);
    auto mpit_cur = mmpMappts.find(m.trainIdx);
    // 合并: 当前帧 -> 参考帧
    if (mpit_ref != ref->mmpMappts.end()) {
      if (mpit_cur != mmpMappts.end()) {
        mpit_ref->second->merge(mpit_ref->second, mpit_cur->second);
      } else {
        mpit_ref->second->add_obs(shared_this, m.trainIdx);
      }
      mmpMappts[m.trainIdx] = mpit_ref->second;
    } else {
      // 合并: 参考帧 -> 当前帧
      if (mpit_cur == mmpMappts.end()) {
        mpit_cur = mmpMappts.insert({m.trainIdx, mpSystem->get_cur_map()->create_mappoint()}).first;
        mpit_cur->second->add_obs(shared_this, m.trainIdx);
      }
      mpit_cur->second->add_obs(ref, m.queryIdx);
      ref->mmpMappts[m.queryIdx] = mpit_cur->second;
    }
  }
  return n;
}


void Frame::mark_keyframe() {
  if (mIdKey > KEY_COUNT) return;
  mIdKey = ++KEY_COUNT;
  for (auto &pair: mmpMappts) {
    if (pair.second->is_invalid()) {
      pair.second->clear();
    } else { mnMappts++; }
  }
  mImg1 = cv::Mat();
  mvUnprojs1.clear();
  mvUnprojs1.shrink_to_fit();
}


void Frame::show_in_opengl(float imu_size, const float *imu_color, bool show_cam) {
  // Frame
  glColor3fv(imu_color);
  pangolin::OpenGlMatrix Tcw = mPose.T_imu_world.matrix();
  pangolin::draw_imu(Tcw, imu_size);
  // camera
  if (show_cam && mpSystem->mpTracker->is_stereo()) {
    const Sophus::SE3f &T_imu_world = mPose.T_imu_world,
        T_cam0_world = T_imu_world * mpSystem->mpTracker->mpCam0->T_cam_imu,
        T_cam1_world = T_imu_world * mpSystem->mpTracker->mpCam1->T_cam_imu;
    glColor3f(0.f, 1.f, 1.f);
    pangolin::draw_imu(T_cam0_world.matrix(), imu_size);
    glColor3f(0.f, 0.f, 0.f);
    pangolin::draw_imu(T_cam1_world.matrix(), imu_size);
  }
}


// Map
Mappoint::Ptr Map::create_mappoint() {
  auto pMappt = std::shared_ptr<Mappoint>(new Mappoint(mpSystem));
  parallel::ScopedLock lock(apTmpMappts.mutex);
  apTmpMappts->push_back(pMappt);
  return pMappt;
}


bool Map::insert_keyframe(const std::shared_ptr<Frame> &pKF) {
  if (!apKeyFrames.mutex.try_lock()) {
    mpSystem->mpAtlas->set_sleep();
    return false;
  }
  apKeyFrames->push_back(pKF);
  mpSystem->mpTracker->mpRefFrame = mpSystem->mpTracker->mpCurFrame;
  // 整理临时地图点
  parallel::ScopedLock lock0(apMappts.mutex), lock1(apTmpMappts.mutex);
  apMappts->reserve(apMappts->size() + apTmpMappts->size());
  for (auto &wpMappt: *apTmpMappts) {
    if (wpMappt.expired()) continue;
    auto pMappt = wpMappt.lock();
    if (pMappt->is_invalid()) continue;
    pMappt->prune();
    if (pMappt->apObs->size() < 2) continue;
    apMappts->push_back(wpMappt);
  }
  apKeyFrames.mutex.unlock();
  return true;
}


void Map::prune_mappts(int begin) {
  auto it = std::remove_if(
      apMappts->begin() + begin, apMappts->end(),
      [](const std::weak_ptr<Mappoint> &wpMappt) {
          // weak_ptr 失效 / 观测点不足
          if (wpMappt.expired()) return true;
          auto pMappt = wpMappt.lock();
          return pMappt->apObs->size() > 1;
      });
  apMappts->erase(it, apMappts->end());
}


void Map::local_mapping() {
  // fixme
  parallel::ScopedLock lock(apKeyFrames.mutex);
  auto it_frame_beg = apKeyFrames->begin() + miKeyFrame;
  feature::BundleAdjustment<g2o::LinearSolverEigen> ba(
      mpSystem->mpTracker->mpCam0->T_cam_imu, (*it_frame_beg)->mId, false,
      it_frame_beg, apKeyFrames->end()
  );
  ba.setForceStopFlag(&mpSystem->mpAtlas->mbSleep);
  // 优化并应用结果
  int i = 0;
  ba.reset();
  for (; i < 2; i++) {
    ba.initializeOptimization(0);
    ba.optimize(10);
    ba.outlier_rejection(true);
    // 中途退出
    if (mpSystem->mpAtlas->mbSleep) break;
  }
  if (i > 0) {
    {
      parallel::ScopedLock lock1(mPosesMutex);
      ba.apply_result();
    }
    miKeyFrame = apKeyFrames->size();
    // 清理地图点
    {
      parallel::ScopedLock lock1(apMappts.mutex);
      prune_mappts(miMappt);
      miMappt = apMappts->size();
    }
  }
}


void Map::draw(Frame::Ptr pCurFrame) const {
  Viewer::Ptr &viewer = mpSystem->mpViewer;
  assert(viewer);
  // Current Frame
  auto kf_end = apKeyFrames->end();
  auto &kf_back = *(kf_end - 1);
  size_t kf_size = apKeyFrames->size();
  if (pCurFrame) {
    pCurFrame->show_in_opengl(viewer->imu_size, viewer->lead_color.data());
    if (kf_size) {
      glColor3fv(viewer->trail_color.data());
      glBegin(GL_LINES);
      glVertex3fv(pCurFrame->get_pos().data());
      glVertex3fv(kf_back->get_pos().data());
      glEnd();
    }
  }
  // Frame
  {
    auto it_beg = kf_end - std::min(viewer->trail_size, kf_size);
    if (it_beg == kf_end) return;
    for (auto it = it_beg; it != kf_end; ++it) {
      (*it)->show_in_opengl(viewer->imu_size, viewer->trail_color.data());
    }
    if (it_beg + 1 != kf_end) {
      glColor3fv(viewer->trail_color.data());
      glBegin(GL_LINES);
      glVertex3fv((*it_beg)->get_pos().data());
      for (auto it = it_beg + 1; it != kf_end - 1; ++it) {
        glVertex3fv((*it)->get_pos().data());
        glVertex3fv((*it)->get_pos().data());
      }
      glVertex3fv(kf_back->get_pos().data());
      glEnd();
    }
  }
  // MapPoints
  glPointSize(viewer->mp_size);
  glColor3fv(viewer->mp_color.data());
  glBegin(GL_POINTS);
  {
    parallel::ScopedLock lock(apTmpMappts.mutex);
    for (auto &wpMappt: *apTmpMappts) {
      if (wpMappt.expired()) continue;
      auto pMappt = wpMappt.lock();
      if (pMappt->is_invalid()) continue;
      glVertex3fv(pMappt->mPos.data());
    }
  }
  {
    parallel::ScopedLock lock(apMappts.mutex);
    for (auto it = apMappts->end() - std::min(viewer->trail_size * apTmpMappts->size(), apMappts->size());
         it != apMappts->end(); ++it) {
      if (it->expired()) continue;
      auto pMappt = it->lock();
      if (pMappt->is_invalid()) continue;
      glVertex3fv(pMappt->mPos.data());
    }
  }
  glEnd();
}


// Mappoint
void Mappoint::prune() {
  parallel::ScopedLock lock(apObs.mutex);
  auto begit = std::remove_if(
      apObs->begin(), apObs->end(),
      [](const Observation &obs) { return obs.first.expired(); });
  apObs->erase(begit, apObs->end());
  // 只剩下一个观测, 而且不是深度观测
  if (apObs->size() == 1) {
    auto [wpf, i] = apObs->front();
    float depth;
    if (!wpf.lock()->get_depth(i, depth)) mbBad = true;
  }
}


void Mappoint::add_obs(const Frame::Ptr &pFrame, const int &idx) {
  parallel::ScopedLock lock(apObs.mutex);
  apObs->emplace_back(pFrame, idx);
}


void Mappoint::erase_obs(const Frame::Ptr &pFrame) {
  parallel::ScopedLock lock(apObs.mutex);
  for (auto it = apObs->begin(); it != apObs->end(); ++it) {
    if (it->first.expired()) continue;
    if (it->first.lock() == pFrame) {
      apObs->erase(it);
      auto itf = pFrame->mmpMappts.find(it->second);
      if (itf->second.get() == this) pFrame->mmpMappts.erase(itf);
      return;
    }
  }
}


void Mappoint::set_pos(const Eigen::Vector3f &pos) {
  mPos = pos;
  mbBad = false;
}


void Mappoint::clear() {
  parallel::ScopedLock lock(apObs.mutex);
  for (auto [wpf, i]: *apObs) {
    if (wpf.expired()) continue;
    auto pf = wpf.lock();
    auto itf = pf->mmpMappts.find(i);
    if (itf->second.get() == this) pf->mmpMappts.erase(itf);
  }
  apObs->clear();
}


void Mappoint::merge(Mappoint::Ptr &shared_this, Mappoint::Ptr &other) {
  parallel::ScopedLock lock0(apObs.mutex), lock1(other->apObs.mutex);
  apObs->insert(apObs->end(), other->apObs->begin(), other->apObs->end());
  for (auto &[wpf, idx]: *other->apObs) {
    if (wpf.expired()) continue;
    wpf.lock()->mmpMappts[idx] = shared_this;
  }
  other->apObs->clear();
  if (mbBad && !other->mbBad) set_pos(other->mPos);
}


// optimize
template<template<typename> class LinearSolverTp>
std::mutex BundleAdjustment<LinearSolverTp>::mtxIdVex;


bool optimize_pose(System *pSystem, const Frame::Ptr &pFrame, const Frame::Ptr &pRefFrame, bool only_pose) {
  std::vector<std::shared_ptr<Frame>> vpFrames = {pFrame};
  if (!only_pose) vpFrames.push_back(pRefFrame);

  // 优化对象: 当前帧, 临时地图点
  BundleAdjustment<g2o::LinearSolverDense> ba(
      pSystem->mpTracker->mpCam0->T_cam_imu, pRefFrame->mId, only_pose,
      vpFrames.begin(), vpFrames.end());

  // 去除无效点; 粗筛外点, 去除负深度点; 精筛外点; 精化位姿
  for (int i = 0; i < 4; ++i) {
    // fail: 关键点过少
    if (ba.edges().size() < pSystem->mpTracker->MIN_MATCHES) return false;
    if (i < 3) ba.reset();
    if (!ba.initializeOptimization(0)) return false;
    ba.optimize(10);
    ba.outlier_rejection(i >= 1);
    pSystem->set_desc("ba-edges", ba.edges().size());
  }
  ba.apply_result();
  return true;
}

}

}
