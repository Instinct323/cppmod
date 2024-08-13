#include <boost/format.hpp>
#include <memory>

#include "utils/cv.hpp"
#include "utils/eigen.hpp"
#include "zjcv/slam.hpp"

#define GRID_SIZE 16

namespace slam::feature {


bool Frame::monocular_init(float &ref_radio, Frame::Ptr pCurFrame) {
  Tracker::Ptr &pTracker = mpSystem->mpTracker;
  bool ok;

  cv::GridDict grid_dict(mvKps0.begin(), mvKps0.end(), mImg0.size(), {GRID_SIZE, GRID_SIZE});
  cv::Mat_<uchar> mask(mpRefFrame->mvKps0.size(), mvKps0.size(), uchar(0));
  for (int i = 0; i < mvKps0.size(); i++) {
    cv::Point2f &pt = mvKps0[i].pt;
    if (pt.x < 0 || pt.x >= mImg0.cols || pt.y < 0 || pt.y >= mImg0.rows) continue;
    grid_dict(pt.x, pt.y, 0).copyTo(mask.row(i));
  }

  pTracker->mpMatcher->search(mpRefFrame->mDesc0, mDesc0, mask, mRefToThisMatches);
  cv::make_one2one(mRefToThisMatches, true);
  ok = mRefToThisMatches.size() > pTracker->MIN_MATCHES;

  if (ok) {
    // 为地图点添加观测
    connect_frame(pCurFrame, mpRefFrame, mRefToThisMatches);
    ok = optimize_pose(mpSystem, pCurFrame, mpRefFrame, false);
    // 计算双目观测的余弦夹角
    if (ok) {
      int invalid = 0;
      for (auto &m: mRefToThisMatches) {
        const Eigen::Vector3f &P0_world = mpRefFrame->mvUnprojs0[m.queryIdx], &P1_cur = mvUnprojs0[m.trainIdx];
        Eigen::Vector3f P0_cur = mPose.T_world_imu * P0_world;
        if (Eigen::cos(P0_cur, P1_cur) > STEREO_OBS_COS_THRESH) invalid++;
      }
      // 无效匹配比例
      ref_radio = float(invalid) / mRefToThisMatches.size();
    }
  }
  return ok;
}


void Frame::match_stereo(int lap_cnt0) {
  Tracker::Ptr &pTracker = mpSystem->mpTracker;

  int lap_cnt1 = pTracker->mpExtractor1->detect_and_compute(mImg1, cv::noArray(), mvKps1, mDesc1);
  pTracker->mpCam1->undistort(mvKps1, mvKps1);

  if (lap_cnt0 > 0 && lap_cnt1 > 0) {
    cv::Mat mask;
    // 分配右相机特征到行
    if (pTracker->mpCam0->get_type() == camera::PINHOLE) {
      mask = cv::Mat_<uchar>(lap_cnt0, lap_cnt1);
      cv::HoriDict hori_dict(mvKps1.begin(), mvKps1.begin() + lap_cnt1, mImg1.rows);
      for (int i = 0; i < lap_cnt0; ++i) hori_dict(mvKps0[i].pt.y).copyTo(mask.row(i));
    }
    // Lowe's ratio test
    std::vector<cv::DMatch> stereo_matches;
    pTracker->mpMatcher->search_with_lowe(mDesc0.rowRange(0, lap_cnt0), mDesc1.rowRange(0, lap_cnt1),
                                          mask, stereo_matches);
    // unproject
    mvUnprojs1.reserve(mvKps1.size());
    for (auto &i: mvKps1) mvUnprojs1.push_back(pTracker->mpCam1->unproject(i.pt));
    // 水平对齐约束, 离群点筛除
    cv::drop_last(stereo_matches, 0.1);
    cv::make_one2one(stereo_matches, true);
    cv::cosine_filter(mvUnprojs0, mvUnprojs1, stereo_matches, 0.9848);
    stereo_triangulation(pTracker->mpCurFrame, stereo_matches);

    mvKps1.clear();
    mvKps1.shrink_to_fit();
    mDesc1.release();
  }
}


bool Frame::match_previous(float &ref_radio) {
  Tracker::Ptr &pTracker = mpSystem->mpTracker;
  const camera::Base::Ptr &pCam0 = pTracker->mpCam0;

  // 利用已有地图点进行匹配
  cv::GridDict grid_dict(mvKps0.begin(), mvKps0.end(), mImg0.size(), {GRID_SIZE, GRID_SIZE});
  Sophus::SE3f T_world_cam0 = pCam0->T_cam_imu.inverse() * mPose.T_world_imu;
  for (int d = 0; d <= 2; d++) {
    cv::Mat_<uchar> mask(mpRefFrame->mvKps0.size(), mvKps0.size(), uchar(0));

    // 将地图点投影到当前帧
    for (auto &pair: mpRefFrame->mmpMappts) {
      if (pair.second->is_invalid()) continue;
      pair.second->prune();
      cv::Point2f pt = pCam0->project(T_world_cam0 * pair.second->mPos);
      if (pt.x < 0 || pt.x >= mImg0.cols || pt.y < 0 || pt.y >= mImg0.rows) continue;
      grid_dict(pt.x, pt.y, d).copyTo(mask.row(pair.first));
    }

    mRefToThisMatches.clear();
    pTracker->mpMatcher->search(mpRefFrame->mDesc0, mDesc0, mask, mRefToThisMatches);
    if (mRefToThisMatches.empty()) LOG(WARNING) << "No matches found";
    cv::drop_last(mRefToThisMatches, 0.1);
    cv::make_one2one(mRefToThisMatches, true);

    ref_radio = float(mRefToThisMatches.size()) / mpRefFrame->mnMappts;
    if (mRefToThisMatches.size() > pTracker->MIN_MATCHES) return true;
  }

  // todo: Relocalization: 重定位

  return mRefToThisMatches.size() > pTracker->MIN_MATCHES;
}


void Frame::process() {
  Tracker::Ptr &pTracker = mpSystem->mpTracker;
  Ptr &pLastFrame = pTracker->mpLastFrame;
  Ptr &pCurFrame = pTracker->mpCurFrame;
  const camera::Base::Ptr &pCam0 = pTracker->mpCam0;

  bool is_keyframe = !mpRefFrame;
  mpSystem->set_desc("id", mId);

  // Initialize: 初始化
  int lap_cnt0 = pTracker->mpExtractor0->detect_and_compute(mImg0, cv::noArray(), mvKps0, mDesc0);
  pCam0->undistort(mvKps0, mvKps0);
  mvUnprojs0.reserve(mvKps0.size());
  for (auto &i: mvKps0) mvUnprojs0.push_back(pCam0->unproject(i.pt));
  bool ok = mvKps0.size() >= pTracker->MIN_MATCHES;

  // motion model: 初始化位姿
  if (pLastFrame) {
    if (pTracker->is_inertial()) {
      mPose.predict_from(mpRefFrame->mPose, pTracker->mpIMUpreint.get());
    } else {
      mPose.predict_from(pLastFrame->mPose);
    }
  } else {
    mPose.set_zero();
  }

  // feature matching: 修正位姿
  if (ok && mpRefFrame) {
    mpSystem->set_desc("n-mappts", mpRefFrame->mnMappts);
    float ref_radio = 0.;

    // Monocular: 第一帧关键帧
    if (pTracker->is_monocular() && mpSystem->get_cur_map()->apKeyFrames->size() == 1) {
      ok = monocular_init(ref_radio, pCurFrame);

    } else {
      // 与参考帧进行匹配
      ok = match_previous(ref_radio);
      if (ok) {
        // 为地图点添加观测
        connect_frame(pCurFrame, mpRefFrame, mRefToThisMatches);
        ok = optimize_pose(mpSystem, pCurFrame, mpRefFrame, true);
      }
    }

    mpSystem->set_desc("ref-radio", (boost::format("%.2f") % ref_radio).str());
    // 根据位姿信息更新
    if (ok) {
      if (!pTracker->is_inertial()) mPose.update_velocity(pLastFrame->mPose);
      // Keyframe: 无效匹配比例 / 匹配成功率 衰减到临界
      if (ref_radio < pTracker->KEY_MATCHES_RADIO) is_keyframe = true;
    }
  }

  // RECENTLY_LOST
  if (!ok) {
    pTracker->switch_state(TrackState::RECENTLY_LOST);
    return;
  }

  // Keyframe: 信息扩充
  if (is_keyframe && mpSystem->get_cur_map()->insert_keyframe(pCurFrame)) {
    // Stereo: 检测右相机特征, 三角化地图点
    if (pTracker->is_stereo()) {
      match_stereo(lap_cnt0);

    } else if (mpRefFrame) {
      // todo Monocular: 清理地图点, 扩充地图点

    }

    // 标记为关键帧
    mark_keyframe();

  } else if (mpRefFrame) {
    // Common Frame: 修改位姿依赖 T_ref_cur = T_world_cur * T_ref_world
    mJoint = Sophus::Joint(&mpRefFrame->mPose.T_world_imu,
                           mPose.T_world_imu * mpRefFrame->mPose.T_imu_world);
  }
  pTracker->switch_state(TrackState::OK);
}


void Frame::draw() {
  Tracker::Ptr pTracker = mpSystem->mpTracker;
  // Image 0
  cv::Mat img0 = mImg0.clone();
  pTracker->mpCam0->undistort(img0, img0);
  cv::cvtColor(img0, img0, cv::COLOR_GRAY2BGR);
  // Reference
  if (mpRefFrame) {
    cv::Mat img1 = mpRefFrame->mImg0.clone(), toshow;
    pTracker->mpCam0->undistort(img1, img1);
    cv::cvtColor(img1, img1, cv::COLOR_GRAY2BGR);
    cv::drawMatches(img1, mpRefFrame->mvKps0, img0, mvKps0, mRefToThisMatches, toshow);
    cv::imshow("Reference", toshow);
  }
}

}
