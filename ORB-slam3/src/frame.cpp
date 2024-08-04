#include <boost/format.hpp>

#include "utils/eigen.hpp"

#define ZJCV_ORB_SLAM

#include "zjcv/slam.hpp"

namespace slam {


void Frame::process() {
  Tracker::Ptr &pTracker = mpSystem->mpTracker;
  Ptr &pLastFrame = pTracker->mpLastFrame;
  Ptr &pCurFrame = pTracker->mpCurFrame;
  Ptr &pRefFrame = pTracker->mpRefFrame;
  camera::Base::Ptr pCam0 = pTracker->mpCam0, pCam1 = pTracker->mpCam1;

  int lap_cnt0, lap_cnt1;
  bool is_keyframe = !pRefFrame;
  mpSystem->set_desc("id", mId);

  // Initialize: 初始化
  {
    // ORB 特征点提取, 去畸, 反投影
    lap_cnt0 = pTracker->mpExtractor0->detect_and_compute(mImg0, cv::noArray(), mvKps0, mDesc0);
    pCam0->undistort(mvKps0, mvKps0);
    mvUnprojs0.reserve(mvKps0.size());
    for (auto &i: mvKps0) mvUnprojs0.push_back(pCam0->unproject(i.pt));
    mvpMappts = std::vector<std::shared_ptr<Mappoint>>(mvUnprojs0.size(), nullptr);

    // RECENTLY_LOST: 关键点过少
    if (mvKps0.size() < pTracker->MIN_MATCHES) {
      pTracker->switch_state(TrackState::RECENTLY_LOST);
      return;
    }

    // motion model: 初始化位姿
    if (pTracker->is_inertial()) {
      if (pRefFrame) mPose.predict_from(pRefFrame->mPose, pTracker->mpIMUpreint.get());
    } else if (pLastFrame) {
      pLastFrame->update_pose();
      mPose.predict_from(pLastFrame->mPose);
    }
  }

  // feature matching: 修正位姿
  if (pRefFrame) {
    pRefFrame->prune();
    auto &ref_mappts = pRefFrame->mvpMappts;

    // 利用已有地图点进行匹配
    cv::GridDict grid_dict(mvKps0.begin(), mvKps0.end(), mImg0.size(), {32, 32});
    std::vector<cv::DMatch> ref_matches;
    cv::Mat_<uchar> mask(pRefFrame->mvKps0.size(), mvKps0.size(), uchar(0));

    // 将地图点投影到当前帧
    Sophus::SE3f T_cam0_world = pCam0->T_cam_imu * mPose.T_imu_world;
    for (int i = 0; i < ref_mappts.size(); ++i) {
      if (!ref_mappts[i] || ref_mappts[i]->is_invalid()) continue;
      cv::Point2f pt = pCam0->project(T_cam0_world * ref_mappts[i]->mPos);
      if (pt.x < 0 || pt.x >= mImg0.cols || pt.y < 0 || pt.y >= mImg0.rows) continue;
      grid_dict(pt.x, pt.y).copyTo(mask.row(i));
    }

    // 余弦相似度筛选
    pTracker->mpMatcher->search(pRefFrame->mDesc0, mDesc0, mask, ref_matches);
    cv::drop_last(ref_matches, 0.1);
    cv::make_one2one(ref_matches, true);
    // cv::cosine_filter(pRefFrame->mvUnprojs0, mvUnprojs0, ref_matches, 0.9397);
    mpSystem->set_desc("ref-match", ref_matches.size());

    // Relocalization: 重定位
    if (ref_matches.size() < pTracker->MIN_MATCHES) {
      ref_matches.clear();
      // todo: 重定位
      if (ref_matches.size() < pTracker->MIN_MATCHES) {
        // RECENTLY_LOST: 匹配点过少
        pTracker->switch_state(TrackState::RECENTLY_LOST);
        return;
      }
    }

    // Keyframe: 匹配点衰减到临界
    if (ref_matches.size() < pTracker->KEY_MATCHES) {
      is_keyframe = true;
    }

    // 为地图点添加观测
    connect_frame(pCurFrame, pRefFrame, ref_matches);
    bool ok = optimize_pose(pCurFrame, pRefFrame);
    if (ok) {
      // Monocular: 上一帧是关键帧, 扩充地图点
      if (pLastFrame->is_keyframe() && pTracker->is_monocular()) {
        // todo
      }

      // 根据位姿信息更新
      pLastFrame->update_pose();
      mPose.update_velocity(pLastFrame->mPose);
      mJoint = Sophus::Joint(&pRefFrame->mPose.T_imu_world, mPose.T_imu_world * pRefFrame->mPose.T_world_imu);
    } else {
      pTracker->switch_state(TrackState::RECENTLY_LOST);
    }
  }

  // Keyframe: 信息扩充
  if (is_keyframe) {

    // Stereo: 检测右相机特征, 三角化地图点
    if (pTracker->is_stereo()) {
      lap_cnt1 = pTracker->mpExtractor1->detect_and_compute(mImg1, cv::noArray(), mvKps1, mDesc1);
      pTracker->mpCam1->undistort(mvKps1, mvKps1);
      if (lap_cnt0 > 0 && lap_cnt1 > 0) {
        // 分配右相机特征到行
        cv::Mat mask;
        if (pTracker->mpCam0->get_type() == camera::PINHOLE) {
          mask = cv::Mat_<uchar>(lap_cnt0, lap_cnt1);
          cv::HoriDict hori_dict(mvKps1.begin(), mvKps1.begin() + lap_cnt1, mImg1.rows);
          for (int i = 0; i < lap_cnt0; ++i) hori_dict(mvKps0[i].pt.y).copyTo(mask.row(i));
        }
        // Lowe's ratio test
        std::vector<cv::DMatch> stereo_matches;
        pTracker->mpMatcher->search_with_lowe(mDesc0.rowRange(0, lap_cnt0), mDesc1.rowRange(0, lap_cnt1),
                                              mask, stereo_matches, 0.7);
        // unproject
        mvUnprojs1.reserve(mvKps1.size());
        for (auto &i: mvKps1) mvUnprojs1.push_back(pCam1->unproject(i.pt));
        // 水平对齐约束, 离群点筛除
        cv::drop_last(stereo_matches, 0.1);
        cv::make_one2one(stereo_matches, true);
        cv::cosine_filter(mvUnprojs0, mvUnprojs1, stereo_matches, 0.9848);
        stereo_matches.shrink_to_fit();
        mStereoMatches = stereo_matches;

        stereo_triangulation(pCurFrame, stereo_matches);
        mpSystem->set_desc("stereo-match", stereo_matches.size());
      }
    }

    // 标记为关键帧
    pTracker->mpRefFrame = pCurFrame;
    mpSystem->get_cur_map()->insert_keyframe(pCurFrame);
    mpSystem->set_desc("id-key", mIdKey);
  }
  pTracker->switch_state(TrackState::OK);
}


void Frame::draw() {
  Tracker::Ptr pTracker = mpSystem->mpTracker;
  cv::Mat img0 = mImg0.clone();
  // Image 0
  pTracker->mpCam0->undistort(img0, img0);
  cv::cvtColor(img0, img0, cv::COLOR_GRAY2BGR);
  std::vector<cv::KeyPoint> kps;
  for (int i = 0; i < mvKps0.size(); ++i) {
    if (mvpMappts[i] && !mvpMappts[i]->is_invalid()) kps.push_back(mvKps0[i]);
  }
  cv::drawKeypoints(img0, kps, img0);
  cv::imshow("Image", img0);
  // Stereo
  if (!mStereoMatches.empty()) {
    cv::Mat img1 = mImg1.clone(), toshow;
    pTracker->mpCam1->undistort(img1, img1);
    cv::cvtColor(img1, img1, cv::COLOR_GRAY2BGR);
    cv::drawMatches(img0, mvKps0, img1, mvKps1, mStereoMatches, toshow);
    cv::imshow("Stereo", toshow);
  }
}

}
