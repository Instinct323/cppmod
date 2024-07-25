#include "utils/eigen.hpp"

#define ZJCV_ORB_SLAM

#include "zjcv/slam.hpp"

namespace slam {


void Frame::process() {
  Tracker::Ptr pTracker = mpSystem->mpTracker;
  Ptr &pLastFrame = pTracker->mpLastFrame;
  Ptr &pCurFrame = pTracker->mpCurFrame;
  Ptr &pRefFrame = pTracker->mpRefFrame;
  camera::Base::Ptr pCam0 = pTracker->mpCam0, pCam1 = pTracker->mpCam1;

  // motion model, 初始化位姿
  if (pTracker->is_inertial()) {
    if (pRefFrame) mPose.predict_from(pRefFrame->mPose, pTracker->mpIMUpreint.get());
  } else if (pLastFrame) {
    mPose.predict_from(pLastFrame->mPose);
  }

  // 特征点提取、去畸
  if (pTracker->is_monocular()) {
    pCam0->monoORBfeatures(pTracker->mpExtractor0.get(), mImg0, mvKps0, mDesc0);
  } else {
    pCam0->stereoORBfeatures(pTracker->mpCam1.get(), pTracker->mpExtractor0.get(), pTracker->mpExtractor1.get(),
                             mImg0, mImg1, mvKps0, mvKps1, mDesc0, mDesc1, mStereoMatches);
  }
  init_mappoints();

  // 与关键帧进行匹配
  if (pRefFrame) {
    std::vector<cv::DMatch> match_ref;
    auto matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    cv::Mat_<uchar> mask(mvKps0.size(), pRefFrame->mvKps0.size());
    for (int i = 0; i < mvKps0.size(); i++) {
      cv::Point2f &pt = mvKps0[i].pt;
      pRefFrame->mpGridMask->operator()(pt.x, pt.y).copyTo(mask.row(i));
    }
    matcher->match(mDesc0, pRefFrame->mDesc0, match_ref, mask);
    // 为地图点添加观测
    std::sort(match_ref.begin(), match_ref.end(),
              [](const cv::DMatch &a, const cv::DMatch &b) { return a.distance < b.distance; });
    connect_frame(pRefFrame, match_ref);
    // todo: 优化位姿, 然后更新速度
    mPose.update_velocity(pLastFrame->mPose);
  }

  // fixme: 仅在关键帧调用
  mpGridMask = std::make_unique<cv::GridMask>(mvKps0, mImg0.size());
  mpSystem->mpAtlas->mpCurMap->insert_keyframe(pCurFrame);
  if (!pRefFrame) pTracker->mpRefFrame = pCurFrame;
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
