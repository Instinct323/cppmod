#include <boost/format.hpp>
#include <memory>

#include "utils/eigen.hpp"

#define ZJCV_ORB_SLAM

#include "zjcv/slam.hpp"

namespace slam {


bool Frame::match_previous(float &radio) {
  Tracker::Ptr &pTracker = mpSystem->mpTracker;
  auto &ref_mappts = mpRefFrame->mvpMappts;
  const camera::Base::Ptr &pCam0 = pTracker->mpCam0;

  // 利用已有地图点进行匹配
  cv::GridDict &grid_dict = *mpRefFrame->mpGridDict;
  for (int d = 0; d <= 2; d++) {
    cv::Mat_<uchar> mask(mpRefFrame->mvKps0.size(), mvKps0.size(), uchar(0));

    // 将地图点投影到当前帧
    int n = 0;
    Sophus::SE3f T_cam0_world = pCam0->T_cam_imu * mPose.T_imu_world;
    for (int i = 0; i < ref_mappts.size(); ++i) {
      if (!ref_mappts[i] || ref_mappts[i]->is_invalid()) continue;
      cv::Point2f pt = pCam0->project(T_cam0_world * ref_mappts[i]->mPos);
      if (pt.x < 0 || pt.x >= mImg0.cols || pt.y < 0 || pt.y >= mImg0.rows) continue;
      n++;
      grid_dict(pt.x, pt.y, d).copyTo(mask.row(i));
    }

    // 余弦相似度筛选
    pTracker->mpMatcher->search(mpRefFrame->mDesc0, mDesc0, mask.t(), mRefToThisMatches);
    cv::cosine_filter(mpRefFrame->mvUnprojs0, mvUnprojs0, mRefToThisMatches, 0.8660);
    cv::make_one2one(mRefToThisMatches, true);
    radio = float(mRefToThisMatches.size()) / n;

    if (mRefToThisMatches.size() > pTracker->MIN_MATCHES) return true;
  }

  // todo: Relocalization: 重定位

  return mRefToThisMatches.size() > pTracker->MIN_MATCHES;
}


void Frame::process() {
  Tracker::Ptr &pTracker = mpSystem->mpTracker;
  Ptr &pLastFrame = pTracker->mpLastFrame;
  Ptr &pCurFrame = pTracker->mpCurFrame;
  mpRefFrame = pTracker->mpRefFrame;
  const camera::Base::Ptr &pCam0 = pTracker->mpCam0, &pCam1 = pTracker->mpCam1;

  int lap_cnt0, lap_cnt1;
  bool is_keyframe = !mpRefFrame;
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
      if (mpRefFrame) mPose.predict_from(mpRefFrame->mPose, pTracker->mpIMUpreint.get());
    } else if (pLastFrame) {
      pLastFrame->update_pose();
      mPose.predict_from(pLastFrame->mPose);
    }
  }

  // feature matching: 修正位姿
  if (mpRefFrame) {
    mpRefFrame->prune();
    bool ok;
    float ref_radio = 1.;

    // Monocular: 第一帧关键帧
    if (pTracker->is_monocular() && mpSystem->get_cur_map()->apKeyFrames->size() == 1) {
      cv::GridDict &grid_dict = *mpRefFrame->mpGridDict;
      cv::Mat_<uchar> mask(mpRefFrame->mvKps0.size(), mvKps0.size(), uchar(0));
      for (int i = 0; i < mvKps0.size(); i++) {
        cv::Point2f &pt = mvKps0[i].pt;
        grid_dict(pt.x, pt.y, 0).copyTo(mask.row(i));
      }
      // 余弦相似度筛选
      pTracker->mpMatcher->search(mpRefFrame->mDesc0, mDesc0, mask.t(), mRefToThisMatches);
      cv::cosine_filter(mpRefFrame->mvUnprojs0, mvUnprojs0, mRefToThisMatches, 0.8660);
      cv::make_one2one(mRefToThisMatches, true);
      // 为地图点添加观测
      connect_frame(pCurFrame, mpRefFrame, mRefToThisMatches);
      ok = optimize_pose(pCurFrame, mpRefFrame, false);
      // 计算双目观测的余弦夹角
      if (ok) {
        int invalid = 0;
        const Sophus::SE3f &T_cur_world = mPose.T_imu_world;
        for (auto &m: mRefToThisMatches) {
          const Eigen::Vector3f &P0_world = mpRefFrame->mvUnprojs0[m.queryIdx], &P1_cur = mvUnprojs0[m.trainIdx];
          Eigen::Vector3f P0_cur = T_cur_world * P0_world;
          if (Eigen::cos(P0_cur, P1_cur) > STEREO_OBS_COS_THRESH) invalid++;
        }
        // 无效匹配比例
        ref_radio = float(invalid) / mRefToThisMatches.size();
      }

    } else {
      // 与参考帧进行匹配
      ok = match_previous(ref_radio);
      if (ok) {
        // 为地图点添加观测
        connect_frame(pCurFrame, mpRefFrame, mRefToThisMatches);
        ok = optimize_pose(pCurFrame, mpRefFrame, true);
      }
    }

    // Keyframe: 无效匹配比例 / 匹配成功率 衰减到临界
    if (ref_radio < pTracker->KEY_MATCHES_RADIO) is_keyframe = true;
    mpSystem->set_desc("ref-radio", (boost::format("%.2f") % ref_radio).str());
    // 根据位姿信息更新
    if (ok) {
      pLastFrame->update_pose();
      mPose.update_velocity(pLastFrame->mPose);
      mJoint = Sophus::Joint(&mpRefFrame->mPose.T_imu_world, mPose.T_imu_world * mpRefFrame->mPose.T_world_imu);
    }

    // RECENTLY_LOST: 关键点过少
    if (!ok) {
      pTracker->switch_state(TrackState::RECENTLY_LOST);
      return;
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

    } else if (mpRefFrame) {
      // todo Monocular: 扩充地图点

    }

    // 标记为关键帧
    pTracker->mpRefFrame = pCurFrame;
    mpSystem->get_cur_map()->insert_keyframe(pCurFrame);
    mpGridDict = std::make_unique<cv::GridDict>(mvKps0.begin(), mvKps0.end(), mImg0.size(), cv::Size(16, 16));
    mpSystem->set_desc("id-key", mIdKey);
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
