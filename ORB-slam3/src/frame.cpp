#include <boost/format.hpp>

#include "utils/eigen.hpp"

#define ZJCV_ORB_SLAM

#include "zjcv/slam.hpp"

namespace slam {


void Frame::unproject_kps() {
  Tracker::Ptr pTracker = mpSystem->mpTracker;
  camera::Base::Ptr pCam0 = pTracker->mpCam0, pCam1 = pTracker->mpCam1;

  assert(mvUnprojs0.empty() && mvUnprojs1.empty());
  mvUnprojs0.reserve(mvKps0.size());
  mvUnprojs1.reserve(mvKps1.size());

  for (auto &i: mvKps0) {
    mvUnprojs0.push_back(pCam0->unproject(i.pt));
  }
  for (auto &i: mvKps1) {
    mvUnprojs1.push_back(pCam1->unproject(i.pt));
  }
}


void Frame::stereo_features(std::vector<cv::DMatch> &matches) {
  Tracker::Ptr &pTracker = mpSystem->mpTracker;
  int lapCnt0, lapCnt1;
  parallel::PriorityThread t0 = parallel::thread_pool.emplace(
      1, [&lapCnt0, this, &pTracker]() {
          lapCnt0 = pTracker->mpExtractor0->detect_and_compute(mImg0, cv::noArray(), mvKps0, mDesc0); \
          pTracker->mpCam0->undistort(mvKps0, mvKps0);
      });
  lapCnt1 = pTracker->mpExtractor1->detect_and_compute(mImg1, cv::noArray(), mvKps1, mDesc1);
  pTracker->mpCam1->undistort(mvKps1, mvKps1);
  t0->join();
  if (lapCnt1 == 0 || lapCnt0 == 0) return;
  // 分配右相机特征到行
  cv::Mat mask;
  if (pTracker->mpCam0->get_type() == camera::PINHOLE) {
    cv::HoriDict hori_dict(mvKps1.begin(), mvKps1.begin() + lapCnt1, mImg1.rows, 2);
    mask = cv::Mat_<uchar>(lapCnt0, lapCnt1);
    for (int i = 0; i < lapCnt0; ++i) hori_dict(mvKps0[i].pt.y).copyTo(mask.row(i));
  }
  // Lowe's ratio test
  pTracker->mpMatcher->search_with_lowe(mDesc0.rowRange(0, lapCnt0), mDesc1.rowRange(0, lapCnt1), cv::Mat(), matches, 0.7);
}


void Frame::process() {
  Tracker::Ptr &pTracker = mpSystem->mpTracker;
  Ptr &pLastFrame = pTracker->mpLastFrame;
  Ptr &pCurFrame = pTracker->mpCurFrame;
  Ptr &pRefFrame = pTracker->mpRefFrame;
  camera::Base::Ptr pCam0 = pTracker->mpCam0, pCam1 = pTracker->mpCam1;

  bool is_keyframe = pRefFrame == nullptr;
  mpSystem->set_desc("id", mId);

  // motion model: 初始化位姿
  if (pTracker->is_inertial()) {
    if (pRefFrame) mPose.predict_from(pRefFrame->mPose, pTracker->mpIMUpreint.get());
  } else if (pLastFrame) {
    mPose.predict_from(pLastFrame->mPose);
  }

  // Monocular: ORB 特征点提取, 去畸, 反投影
  if (pTracker->is_monocular()) {
    pTracker->mpExtractor0->detect_and_compute(mImg0, cv::noArray(), mvKps0, mDesc0);
    pCam0->undistort(mvKps0, mvKps0);
    unproject_kps();

  } else {
    // Stereo
    stereo_features(mStereoMatches);
    unproject_kps();
    // 水平对齐约束, 离群点筛除
    cv::dy_filter(mvUnprojs0, mvUnprojs1, mStereoMatches, 1.5);
    cv::drop_last(mStereoMatches, 0.1);
    mStereoMatches.shrink_to_fit();
  }

  // RECENTLY_LOST: 关键点过少
  if (mvKps0.size() < pTracker->MIN_MATCHES) {
    pTracker->switch_state(TrackState::RECENTLY_LOST);
    return;
  }

  // LOST: 重定位
  if (pTracker->mState == TrackState::LOST) {
    // todo: 重定位
  }

  // 初始化地图点
  init_mappoints(pCurFrame, mStereoMatches);
  mpSystem->set_desc("stereo-matches", mStereoMatches.size());
  if (pRefFrame) {
    pRefFrame->prune();
    auto &refMappts = pRefFrame->mvpMappts;

    // 利用已有地图点进行匹配
    cv::GridDict grid_dict(mvKps0.begin(), mvKps0.end(), mImg0.size());
    std::vector<cv::DMatch> ref_matches;
    cv::Mat_<uchar> mask(pRefFrame->mvKps0.size(), mvKps0.size(), uchar(0));

    // 将地图点投影到当前帧
    Sophus::SE3f T_cam0_world = pCam0->T_cam_imu * mPose.T_imu_world;
    for (int i = 0; i < refMappts.size(); ++i) {
      if (!refMappts[i] || refMappts[i]->is_invalid()) continue;
      cv::Point2f pt = pCam0->project(T_cam0_world * refMappts[i]->mPos);
      if (pt.x < 0 || pt.x >= mImg0.cols || pt.y < 0 || pt.y >= mImg0.rows) continue;
      grid_dict(pt.x, pt.y).copyTo(mask.row(i));
    }

    // 余弦相似度筛选
    pTracker->mpMatcher->search(pRefFrame->mDesc0, mDesc0, mask, ref_matches);
    float n = ref_matches.size() + 1e-4;
    cv::cosine_filter(pRefFrame->mvUnprojs0, mvUnprojs0, ref_matches, 0.8);
    float r = float(ref_matches.size()) / n;
    cv::drop_last(ref_matches, 0.1);

    mpSystem->set_desc("ref-matches", ref_matches.size());
    mpSystem->set_desc("ref-filter", (boost::format("%.2f") % r).str());

    // RECENTLY_LOST: 匹配点过少
    if (ref_matches.size() < pTracker->MIN_MATCHES) {
      pTracker->switch_state(TrackState::RECENTLY_LOST);
    }

    // Keyframe: 匹配点衰减到临界
    if (ref_matches.size() <= pTracker->KEY_MATCHES) {
      is_keyframe = true;
    }

    // 为地图点添加观测
    connect_frame(pCurFrame, pRefFrame, ref_matches);
    optimize_pose(this);

    // Monocular: 上一帧是关键帧, 扩充地图点
    if (pLastFrame->is_keyframe() && pTracker->is_monocular()) {
      // todo
    }
    mPose.update_velocity(pLastFrame->mPose);
  }

  // 标记为关键帧, 同时三角化已有地图点
  if (is_keyframe) {
    mark_keyframe();
    pTracker->mpRefFrame = pCurFrame;
    mpSystem->mpAtlas->mpCurMap->insert_keyframe(pCurFrame);
    mpSystem->set_desc("id-key", mIdKey);
  }
  pTracker->switch_state(TrackState::OK);
}


void Frame::draw() {
  Tracker::Ptr pTracker = mpSystem->mpTracker;
  cv::Mat img0 = mImg0.clone(), img1 = mImg1.clone(), toshow;
  // Image 0
  pTracker->mpCam0->undistort(img0, img0);
  cv::cvtColor(img0, img0, cv::COLOR_GRAY2BGR);
  if (img1.empty()) {
    cv::drawKeypoints(img0, mvKps0, toshow);
  } else {
    // Image 1
    pTracker->mpCam1->undistort(img1, img1);
    cv::cvtColor(img1, img1, cv::COLOR_GRAY2BGR);
    cv::drawMatches(img0, mvKps0, img1, mvKps1, mStereoMatches, toshow);
  }
  cv::imshow("Image", toshow);
}

}
