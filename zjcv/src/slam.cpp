#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include "utils/sophus.hpp"
#include "zjcv/slam.hpp"

namespace slam {


// Atlas
feature::Map::Ptr Atlas::create_map() {
  mpCurMap = std::make_shared<feature::Map>(mpSystem);
  mvpMaps.push_back(mpCurMap);
  return mpCurMap;
}


void Atlas::run() {
  while (mpSystem->mbRunning) {
    // todo: 调用 Map 类内方法优化关键帧及地图点
    LOG(FATAL) << "Not implemented";
  }
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
    KEY_MATCHES_RADIO(cfg["key_matches_radio"].as<float>()), LOST_TIMEOUT(cfg["lost_timeout"].as<double>()),

    mpCam0(camera::from_yaml(cfg["cam0"])), mpCam1(camera::from_yaml(cfg["cam1"])),
    mpIMU(IMU::Device::from_yaml(cfg["imu"])) {

  ASSERT(!is_inertial(), "Not implemented")
  ASSERT(mpCam0, "Camera0 not found")
  if (is_stereo()) {
    // 校对相机类型
    ASSERT(mpCam0->get_type() == mpCam1->get_type(), "Camera0 and Camera1 must be the same type")
  }
}


void Tracker::grab_image(const double &timestamp, const cv::Mat &img0, const cv::Mat &img1) {
  ASSERT(!is_inertial() || timestamp <= mpIMUpreint->mtEnd, "Grab image before IMU data is integrated")
  ASSERT(is_monocular() == img1.empty(), "Invalid image pair")
  mpLastFrame = mpCurFrame;
  mpCurFrame = std::make_shared<feature::Frame>(mpSystem, timestamp, img0, img1);
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
    mpSystem->mpAtlas->create_map();
  }
}


// Viewer
Viewer::Viewer(System *pSystem, const YAML::Node &cfg
) : mpSystem(pSystem), delay(1000 / cfg["fps"].as<int>()),
    imu_size(cfg["imu_size"].as<float>()), mp_size(cfg["mp_size"].as<float>()), trail_size(cfg["trail_size"].as<size_t>()),
    lead_color(YAML::toEigen<float>(cfg["lead_color"])), trail_color(YAML::toEigen<float>(cfg["trail_color"])),
    mp_color(YAML::toEigen<float>(cfg["mp_color"])) {
  ASSERT(delay > 0, "Viewer: The delay must be greater than 0")
}


// 基于特征点
namespace feature {


// Frame
size_t Frame::FRAME_COUNT = 0;
size_t Frame::KEY_COUNT = 0;


Frame::Frame(System *pSystem, const double &timestamp,
             const cv::Mat &img0, const cv::Mat &img1
) : mpSystem(pSystem), mId(++FRAME_COUNT), mTimestamp(timestamp), mImg0(img0), mImg1(img1),
    mpRefFrame(pSystem->mpTracker->mpRefFrame) {}


int Frame::stereo_triangulation(const Frame::Ptr &shared_this, const std::vector<cv::DMatch> &stereo_matches) {
  int cnt = 0;
  if (!stereo_matches.empty()) {
    const Sophus::SE3f &T_cam0_cam1 = mpSystem->mpTracker->mpCam1->T_cam_imu.inverse() * mpSystem->mpTracker->mpCam0->T_cam_imu,
        Identity,
        T_cam0_world = mPose.T_imu_world * mpSystem->mpTracker->mpCam0->T_cam_imu;

    for (auto m: stereo_matches) {
      int i = m.queryIdx, j = m.trainIdx;
      // 余弦判断
      const Eigen::Vector3f &P0_cam0 = mvUnprojs0[i], &P1_cam1 = mvUnprojs1[j];
      Eigen::Vector3f P1_cam0 = T_cam0_cam1 * P0_cam0;
      if (Eigen::cos(P0_cam0, P1_cam0) > STEREO_OBS_COS_THRESH) continue;
      // 三角化
      Eigen::Vector3f P_cam0;
      float rep_error = Sophus::triangulation({P0_cam0, P1_cam1}, {Identity, T_cam0_cam1}, P_cam0);
      if (rep_error >= 0) {
        cnt += 1;
        mvUnprojs0[i][2] = -P_cam0[2];
        // 创建地图点
        if (!mvpMappts[i]) {
          mvpMappts[i] = mpSystem->get_cur_map()->create_mappoint(mId);
          mvpMappts[i]->add_obs(shared_this, i);
        }
        mvpMappts[i]->set_pos(T_cam0_world * P_cam0);
      }
    }
  }
  return cnt;
}


int Frame::connect_frame(Frame::Ptr &shared_this, Frame::Ptr &ref, std::vector<cv::DMatch> &ref2this) {
  // matches: 参考帧 -> 当前帧
  int n = std::min(int(ref2this.size()), mpSystem->mpTracker->MAX_MATCHES);
  for (int i = 0; i < n; ++i) {
    const cv::DMatch &m = ref2this[i];
    auto mpit_ref = ref->mvpMappts.begin() + m.queryIdx;
    auto mpit_cur = mvpMappts.begin() + m.trainIdx;
    // 合并: 当前帧 -> 参考帧
    if (*mpit_ref) {
      if (*mpit_cur) {
        (*mpit_ref)->merge((*mpit_ref), (*mpit_cur));
      } else {
        (*mpit_ref)->add_obs(shared_this, m.trainIdx);
      }
      *mpit_cur = *mpit_ref;
    } else {
      // 合并: 参考帧 -> 当前帧
      if (!*mpit_cur) {
        *mpit_cur = mpSystem->get_cur_map()->create_mappoint(ref->mId);
        (*mpit_cur)->add_obs(shared_this, m.trainIdx);
      }
      (*mpit_cur)->add_obs(ref, m.queryIdx);
      *mpit_ref = *mpit_cur;
    }
  }
  return n;
}


void Frame::mark_keyframe() {
  if (mIdKey > KEY_COUNT) return;
  mIdKey = ++KEY_COUNT;
  for (Mappoint::Ptr &m: mvpMappts) if (m) mnMappts++;
}


void Frame::prune() { for (Mappoint::Ptr &m: mvpMappts) if (m) m->prune(); }


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
Mappoint::Ptr Map::create_mappoint(size_t id_frame) {
  auto pMappt = std::make_shared<Mappoint>(mpSystem, id_frame);
  parallel::ScopedLock lock(apTmpMappts.mutex);
  apTmpMappts->push_back(pMappt);
  return pMappt;
}


void Map::insert_keyframe(const std::shared_ptr<Frame> &pKF) {
  pKF->mark_keyframe();
  mpSystem->mpTracker->mpRefFrame = mpSystem->mpTracker->mpCurFrame;
  {
    parallel::ScopedLock lock(apKeyFrames.mutex);
    apKeyFrames->push_back(pKF);
  }
  // 整理临时地图点
  parallel::ScopedLock lock0(apMappts.mutex), lock1(apTmpMappts.mutex);
  apMappts->reserve(apMappts->size() + apTmpMappts->size());
  for (auto &pMappt: *apTmpMappts) {
    if (pMappt.expired()) continue;
    apMappts->push_back(pMappt);
  }
}


void Map::prune(int i) {
  parallel::ScopedLock lock(apMappts.mutex);
  auto it = std::remove_if(
      apMappts->begin() + i, apMappts->end(),
      [](const std::weak_ptr<Mappoint> &wpMappt) { return wpMappt.expired(); });
  apMappts->erase(it, apMappts->end());
}


void Map::draw() const {
  Viewer::Ptr &viewer = mpSystem->mpViewer;
  assert(viewer);
  auto it_beg = apKeyFrames->end() - std::min(viewer->trail_size, apKeyFrames->size());
  if (it_beg == apKeyFrames->end()) return;
  // Frame
  {
    parallel::ScopedLock lock0(apKeyFrames.mutex);
    for (auto it = it_beg; it != apKeyFrames->end(); ++it) {
      (*it)->show_in_opengl(viewer->imu_size, viewer->trail_color.data());
    }
    if (it_beg + 1 != apKeyFrames->end()) {
      glBegin(GL_LINES);
      glColor3fv(viewer->trail_color.data());
      glVertex3fv((*it_beg)->get_pos().data());
      for (auto it = it_beg + 1; it != apKeyFrames->end() - 1; ++it) {
        glVertex3fv((*it)->get_pos().data());
        glVertex3fv((*it)->get_pos().data());
      }
      glVertex3fv(apKeyFrames->back()->get_pos().data());
      glEnd();
    }
  }
  // MapPoints
  glPointSize(viewer->mp_size);
  glBegin(GL_POINTS);
  glColor3fv(viewer->mp_color.data());
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
    for (auto &wpMappt: *apMappts) {
      if (wpMappt.expired()) continue;
      auto pMappt = wpMappt.lock();
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
  // 只剩下一个观测, 而且不是双目观测
  if (apObs->size() == 1) {
    auto [wpf, i] = apObs->front();
    float d = wpf.lock()->mvUnprojs0[i][2];
    if (d > 0) mbBad = true;
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
      pFrame->mvpMappts[it->second] = nullptr;
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
    wpf.lock()->mvpMappts[i] = nullptr;
  }
  apObs->clear();
}


void Mappoint::merge(Mappoint::Ptr &shared_this, Mappoint::Ptr &other) {
  parallel::ScopedLock lock0(apObs.mutex), lock1(other->apObs.mutex);
  apObs->insert(apObs->end(), other->apObs->begin(), other->apObs->end());
  for (auto &[wpf, idx]: *other->apObs) {
    if (wpf.expired()) continue;
    wpf.lock()->mvpMappts[idx] = shared_this;
  }
  other->apObs->clear();
  if (!other->mbBad) set_pos(other->mPos);
}


// optimize
bool optimize_pose(System *pSystem, const Frame::Ptr &pFrame, const Frame::Ptr &pRefFrame, bool only_pose) {
  std::vector<std::shared_ptr<Frame>> vpFrames = {pFrame};
  if (!only_pose) vpFrames.push_back(pRefFrame);
  auto &apMappts = pSystem->get_cur_map()->apTmpMappts;

  // 优化对象: 当前帧, 临时地图点
  apMappts.mutex.lock();
  BundleAdjustment<g2o::LinearSolverDense> ba(
      pSystem->mpTracker->mpCam0->T_cam_imu, pRefFrame->mId, only_pose,
      vpFrames.begin(), vpFrames.end(), apMappts->begin(), apMappts->end());
  apMappts.mutex.unlock();

  // fail: 关键点过少
  const int &MIN_MATCHES = pSystem->mpTracker->MIN_MATCHES;

  // 去除无效点; 粗筛外点, 去除负深度点; 精筛外点; 精化位姿
  for (int i = 0; i < 4; ++i) {
    if (ba.edges().size() < MIN_MATCHES) return false;
    if (i < 2) ba.reset();
    if (!ba.initializeOptimization(0)) return false;
    ba.optimize(10);
    ba.outlier_rejection(true, i == 1);
    pSystem->set_desc("ba-edges", ba.edges().size());
  }
  parallel::ScopedLock lock(apMappts.mutex);
  ba.apply_result();
  return true;
}

}

}
