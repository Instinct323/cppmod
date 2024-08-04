#include <g2o/solvers/dense/linear_solver_dense.h>

#include "utils/sophus.hpp"
#include "zjcv/slam.hpp"

namespace slam {


// Atlas
feature::Map::Ptr Atlas::create_map() {
  mpCurMap = std::make_shared<feature::Map>(mpSystem);
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

  ASSERT(!is_inertial(), "Not implemented")
  ASSERT(mpCam0, "Camera0 not found")
  if (is_stereo()) {
    // 校对相机类型
    ASSERT(mpCam0->get_type() == mpCam1->get_type(), "Camera0 and Camera1 must be the same type")
    T_cam0_cam1 = mpCam0->T_cam_imu * mpCam1->T_cam_imu.inverse();
    T_cam1_cam0 = T_cam0_cam1.inverse();
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
    mJoint(&mPose.T_imu_world) {}


void Frame::update_pose() { mPose.set_pose(mJoint.get()); }


int Frame::stereo_triangulation(const Frame::Ptr &shared_this, const std::vector<cv::DMatch> &stereo_matches) {
  int cnt = 0;
  if (!stereo_matches.empty()) {
    Sophus::SE3f T_cam1_cam0 = mpSystem->mpTracker->T_cam1_cam0,
        T_imu_cam0 = mpSystem->mpTracker->mpCam0->T_cam_imu.inverse();
    for (auto m: stereo_matches) {
      int i = m.queryIdx, j = m.trainIdx;
      // 余弦判断
      const Eigen::Vector3f &P0_cam0 = mvUnprojs0[i], &P1_cam1 = mvUnprojs1[j];
      Eigen::Vector3f P0_cam1 = T_cam1_cam0 * P0_cam0;
      if (Eigen::cos(P0_cam0, P0_cam1) > 0.9998) continue;
      // 三角化
      Eigen::Vector3f P_cam0;
      float rep_error = Sophus::triangulation({P0_cam0, P1_cam1}, {Sophus::SE3f(), T_cam1_cam0}, P_cam0);
      if (rep_error >= 0) {
        cnt += 1;
        mvUnprojs0[i][2] = -P_cam0[2];
        // 创建地图点
        if (!mvpMappts[i]) {
          mvpMappts[i] = mpSystem->get_cur_map()->create_mappoint(mId);
          mvpMappts[i]->add_obs(shared_this, i);
        }
        mvpMappts[i]->set_pos(T_imu_cam0 * P_cam0);
      }
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
        *mpit_cur = mpSystem->get_cur_map()->create_mappoint(other->mId);
        (*mpit_cur)->add_obs(shared_this, m.trainIdx);
      }
      (*mpit_cur)->add_obs(other, m.queryIdx);
      *mpit_ref = *mpit_cur;
    }
  }
  return n;
}


void Frame::mark_keyframe() { mIdKey = ++KEY_COUNT; }


void Frame::prune() { for (Mappoint::Ptr &m: mvpMappts) if (m) m->prune(); }


void Frame::show_in_pangolin(float imu_size, float mp_size, const float *imu_color, const float *mp_color) {
  // Frame
  glColor3fv(imu_color);
  pangolin::OpenGlMatrix Twc = mPose.T_world_imu.matrix();
  pangolin::draw_imu(Twc, imu_size);
  // MapPoints
  glPointSize(mp_size);
  for (auto &pMappt: mvpMappts) {
    if (!pMappt || pMappt->is_invalid()) continue;
    glColor3fv(mp_color);
    glBegin(GL_LINES);
    glVertex3fv(mPose.T_imu_world.translation().data());
    glVertex3fv(pMappt->mPos.data());
    glEnd();
    glColor3f(0.f, 0.f, 0.f);
    glBegin(GL_POINTS);
    glVertex3fv(pMappt->mPos.data());
    glEnd();
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


void Map::draw() const {
  Viewer::Ptr &viewer = mpSystem->mpViewer;
  if (!viewer) return;
  size_t n = std::min(viewer->trail_size, apKeyFrames->size());
  // pangolin
  parallel::ScopedLock lock(apKeyFrames.mutex);
  for (auto it = apKeyFrames->end() - n; it != apKeyFrames->end(); ++it) {
    (*it)->show_in_pangolin(viewer->imu_size, viewer->mp_size, viewer->trail_color.data(), viewer->mp_color.data());
  }
}


// Mappoint
void Mappoint::prune() {
  parallel::ScopedLock lock(apObs.mutex);
  auto begit = std::remove_if(
      apObs->begin(), apObs->end(),
      [](const Observation &obs) { return obs.first.expired(); });
  apObs->erase(begit, apObs->end());
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


Mappoint &Mappoint::operator+=(const Mappoint &other) {
  parallel::ScopedLock lock(apObs.mutex);
  apObs->insert(apObs->end(), other.apObs->begin(), other.apObs->end());
  return *this;
}


// optimize
bool optimize_pose(const Frame::Ptr &pFrame, const Frame::Ptr &pRefFrame, int n_iters) {
  std::vector<std::shared_ptr<Frame>> vpFrames = {pFrame};
  auto &apMapppts = pFrame->mpSystem->get_cur_map()->apTmpMappts;
  apMapppts.mutex.lock();
  BundleAdjustment<g2o::LinearSolverDense> ba(pRefFrame->mId, true, vpFrames.begin(), vpFrames.end(), apMapppts->begin(), apMapppts->end());
  apMapppts.mutex.unlock();
  const int &MIN_MATCHES = pFrame->mpSystem->mpTracker->MIN_MATCHES;
  if (ba.valid_mappts() < MIN_MATCHES) return false;

  for (int i = 0; i < n_iters; ++i) {
    ba.reset();
    ba.initializeOptimization(0);
    ba.optimize(15);
    ba.outlier_rejection(5.991f * (n_iters - i));
    if (ba.valid_mappts() < MIN_MATCHES) break;
  }
  parallel::ScopedLock lock(apMapppts.mutex);
  ba.apply_result(false);
  return true;
}

}

}
